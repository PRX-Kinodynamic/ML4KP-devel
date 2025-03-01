#pragma once
#include "prx/utilities/defs.hpp"
#include "prx/mujoco/mj_simulator.hpp"
#include <vector>
#include <memory>
#include <iostream>
#include <limits>
#include "namo_utility.hpp"

namespace prx
{

/**
 * @brief Environment class for Navigation Among Movable Obstacles (NAMO)
 * 
 * Manages the simulation environment, including robot, static obstacles, and movable objects.
 * Provides interfaces for state management, collision checking, and simulation control.
 */
class NAMOEnvironment
{
public:
  /**
   * @brief Structure to store object information
   * 
   * Contains geometric and physical properties of objects in the environment
   */
  struct ObjectInfo {
    int body_id;                    ///< MuJoCo body ID
    int geom_id;                    ///< MuJoCo geometry ID
    std::string name;               ///< Object name
    bool is_static;                 ///< Whether object is static or movable
    // Geometric properties
    std::array<double, 3> position;    // geom_pos
    std::array<double, 3> size;        // geom_size
    std::array<double, 4> quaternion;  // geom_quat 
    mjtGeom geom_type;                 // geom_type

    int symmetry_rotations;

    // Add default constructor
    ObjectInfo() : size({1.0, 1.0, 1.0}), symmetry_rotations(4) {}
    
    ObjectInfo(const std::array<double, 3>& size) : size(size) {
        // If x and y dimensions are within 5% of each other, 4-way symmetry
        double size_ratio = std::max(size[0], size[1]) / std::min(size[0], size[1]);
        symmetry_rotations = (size_ratio < 1.05) ? 4 : 2;
    }
  };
  
  /**
   * @brief Structure to store object state
   * 
   * Contains the current state (position, orientation, velocities) of an object
   */
  struct ObjectState {
    std::string name;                  ///< Object name
    std::array<double, 3> position;    ///< Current position
    std::array<double, 4> quaternion;  ///< Current orientation (quaternion)
    std::array<double, 3> size;        ///< Current size
    std::array<double, 3> linear_vel;  ///< Linear velocity
    std::array<double, 3> angular_vel; ///< Angular velocity
  };

  /**
   * @brief Construct a new NAMO Environment
   * 
   * @param xml_path Path to MuJoCo XML model file
   * @param visualize Whether to enable visualization
   */
  NAMOEnvironment(const std::string& xml_path, bool visualize)
    : sim(std::make_shared<mujoco_simulator_t>(xml_path, visualize))
  {
    sim->init_simulator();
    warm_up();

    auto context = sim->get_context("mujoco");
    sg = context.first;
    ss = sg->get_state_space();
    cs = sg->get_control_space();


    // Store robot information
    robot_id = mj_name2id(sim->m, mjOBJ_GEOM, "robot");
    robot_info.body_id = mj_name2id(sim->m, mjOBJ_BODY, "robot");
    robot_info.geom_id = robot_id;
    robot_info.name = "robot";
    robot_info.is_static = false;
    
    // Store robot geometric properties
    for (int k = 0; k < 3; k++) {
      robot_info.position[k] = sim->m->geom_pos[robot_id * 3 + k];
      robot_info.size[k] = sim->m->geom_size[robot_id * 3 + k];
    }
    for (int k = 0; k < 4; k++) {
      robot_info.quaternion[k] = sim->m->geom_quat[robot_id * 4 + k];
    }

    // Process and categorize objects in the environment
    process_environment_objects();

    // get the bounds of the environment
    std::vector<double> bounds = get_environment_bounds();

    std::vector<double> lower_bounds = ss->get_lower_bounds();
    std::vector<double> upper_bounds = ss->get_upper_bounds();

    lower_bounds[0] = bounds[0];
    lower_bounds[1] = bounds[2];
    upper_bounds[0] = bounds[1];
    upper_bounds[1] = bounds[3];
    ss->set_bounds(lower_bounds, upper_bounds);

    
    // std::vector<double> lower_bounds = {bounds[0], bounds[2]};
    // std::vector<double> upper_bounds = {bounds[1], bounds[3]};  
    // ss->set_bounds(lower_bounds, upper_bounds);

    init_robot_pos = {sim->d->geom_xpos[3 * robot_id], sim->d->geom_xpos[3 * robot_id + 1], sim->d->geom_xpos[3 * robot_id + 2]};
    current_qpos = ss->make_point();

  }

  std::vector<double> get_random_state() {
    // returns a random x, y, yaw in the environment
    std::vector<double> bounds = get_environment_bounds();
    std::vector<double> state(3);
    state[0] = uniform_random(bounds[0], bounds[1]);
    state[1] = uniform_random(bounds[2], bounds[3]);
    state[2] = 0.0;
    return state;
  }

  /**
   * @brief Get list of static objects in the environment
   * @return const std::vector<ObjectInfo>& List of static objects
   */
  const std::vector<ObjectInfo>& get_static_objects() const { return static_objects; }

  /**
   * @brief Get list of movable objects in the environment
   * @return const std::vector<ObjectInfo>& List of movable objects
   */
  const std::vector<ObjectInfo>& get_movable_objects() const { return movable_objects; }

  /**
   * @brief Get information about a specific object
   * 
   * @param name Object name
   * @return const ObjectInfo* Pointer to object info, nullptr if not found
   */
  const ObjectInfo* get_object_info(const std::string& name) const {
    auto it = object_map.find(name);
    return it != object_map.end() ? &(it->second) : nullptr;
  }

  /**
   * @brief Reset the environment to initial state
   */
  void reset()
  {
    sim->reset_simulation();
    warm_up();
    ss->copy_to(current_qpos);
    
    // Initialize object states after reset
    update_object_states();
  }

  /**
   * @brief Step the simulation with given control input
   * 
   * @param control Control input
   * @param duration Duration to apply control
   */
  void step(const space_point_t& control, double duration)
  {
    trajectory_t traj(ss);
    plan_t plan(cs);
    traj.clear();
    plan.clear();

    plan.append_onto_back(duration);
    plan.back().control = control;

    ss->copy_to(current_qpos);

    sg->propagate(current_qpos, plan, traj);
    
    ss->copy_from(traj.back());
    
    // Update object states after simulation step
    update_object_states();
  }

  void step_simulation() {
    sim->step_simulation();
    
    // Update object states after simulation step
    update_object_states();
  }

  /**
   * @brief Check if robot is in collision with any object
   * @return bool True if collision detected
   */
  bool is_in_collision()
  {
    return sim->in_collision();
  }

  /**
   * @brief Get a point in state space
   * @return space_point_t State space point
   */
  space_point_t get_state_space_point()
  {
    return ss->make_point();
  }

  /**
   * @brief Get a point in control space
   * @return space_point_t Control space point
   */
  space_point_t get_control_space_point()
  {
    return cs->make_point();
  }

  /**
   * @brief Get current state of the environment
   * @return space_point_t Current state
   */
  space_point_t get_current_qpos()
  {
    return current_qpos;
  }

  /**
   * @brief Get state space
   * @return space_t* Pointer to state space
   */
  space_t* get_state_space() const
  {
    return ss;
  }

  /**
   * @brief Get control space
   * @return space_t* Pointer to control space
   */
  space_t* get_control_space() const
  {
    return cs;
  }

  /**
   * @brief Get the bounds of the environment
   * @return std::vector<double> Bounds as [x_min, x_max, y_min, y_max]
   */
  std::vector<double> get_environment_bounds() const {
    std::vector<double> bounds = {
      std::numeric_limits<double>::max(),   // x_min
      std::numeric_limits<double>::lowest(), // x_max
      std::numeric_limits<double>::max(),   // y_min
      std::numeric_limits<double>::lowest()  // y_max
    };

    // Include static objects in bounds calculation
    for (const auto& obj : static_objects) {
      double half_width = obj.size[0] / 2.0;
      double half_height = obj.size[1] / 2.0;
      
      // Get rotation angle using utility function
      double angle = quaternion_to_yaw(obj.quaternion);
      
      std::vector<std::pair<double, double>> corners = {
        {-half_width, -half_height},
        {-half_width, half_height},
        {half_width, -half_height},
        {half_width, half_height}
      };
      
      // Update bounds with rotated corners using utility function
      for (const auto& corner : corners) {
        auto rotated = rotate_point(
          corner.first, corner.second,
          0, 0,  // Rotate around origin
          angle
        );
        double rotated_x = rotated[0] + obj.position[0];
        double rotated_y = rotated[1] + obj.position[1];
        
        bounds[0] = std::min(bounds[0], rotated_x);
        bounds[1] = std::max(bounds[1], rotated_x);
        bounds[2] = std::min(bounds[2], rotated_y);
        bounds[3] = std::max(bounds[3], rotated_y);
      }
    }

    // Include movable objects in bounds calculation
    for (const auto& obj : movable_objects) {
      double half_width = obj.size[0] / 2.0;
      double half_height = obj.size[1] / 2.0;
      
      // Get rotation angle using utility function
      double angle = quaternion_to_yaw(obj.quaternion);
      
      std::vector<std::pair<double, double>> corners = {
        {-half_width, -half_height},
        {-half_width, half_height},
        {half_width, -half_height},
        {half_width, half_height}
      };
      
      // Update bounds with rotated corners using utility function
      for (const auto& corner : corners) {
        auto rotated = rotate_point(
          corner.first, corner.second,
          0, 0,  // Rotate around origin
          angle
        );
        double rotated_x = rotated[0] + obj.position[0];
        double rotated_y = rotated[1] + obj.position[1];
        
        bounds[0] = std::min(bounds[0], rotated_x);
        bounds[1] = std::max(bounds[1], rotated_x);
        bounds[2] = std::min(bounds[2], rotated_y);
        bounds[3] = std::max(bounds[3], rotated_y);
      }
    }

    // Add some padding to the bounds
    const double PADDING = 0.5;  // 0.5 meters of padding
    bounds[0] = -4.0;//-= PADDING;
    bounds[1] = 4.0;//+= PADDING;
    bounds[2] = -4.0;//-= PADDING;
    bounds[3] = 4.0;//+= PADDING;

    return bounds;
  }

  /**
   * @brief Get information about the robot
   * @return const ObjectInfo& Reference to robot information
   */
  const ObjectInfo& get_robot_info() const { return robot_info; }


  void set_robot_position(const std::array<double, 2>& pos) {
    // Initial robot positioning
    std::array<double, 3> robot_pos = {
        pos[0] - init_robot_pos[0], 
        pos[1] - init_robot_pos[1], 
        robot_info.size[2]
    };
    sim->d->qpos[0] = robot_pos[0];
    sim->d->qpos[1] = robot_pos[1];
    // sim->d->qpos[2] = robot_pos[2];

    for(int i = 0; i < sim->m->nv; i++) {
      sim->d->qvel[i] = 0.0;
    }
    mj_forward(sim->m, sim->d);
  }

  void set_zero_velocity() {
    for(int i = 0; i < sim->m->nv; i++) {
      sim->d->qvel[i] = 0.0;
    }
    mj_forward(sim->m, sim->d);
    update_object_states();
  }
  
  void set_goal(const MujocoGoal& goal) {
    sim->set_goal(goal);
  }

  /**
   * @brief Update the state of all objects after simulation step
   * 
   * Updates the internal object_states map with current positions and velocities
   */
  void update_object_states() {
    

    // Update movable objects
    for (const auto& obj : movable_objects) {
      ObjectState& state = object_states[obj.name];
      state.name = obj.name;
      
      // Get position from mjData
      for (int i = 0; i < 3; i++) {
        state.position[i] = sim->d->geom_xpos[obj.geom_id * 3 + i];
      }
      
      // Get quaternion from mjData
      mjtNum* obj_quat = sim->d->geom_xmat + obj.geom_id * 9;
      mju_mat2Quat(state.quaternion.data(), obj_quat);
      
      // Get velocities
      if (obj.body_id >= 0) {
        int body_vel_adr = 6 * obj.body_id;
        for (int i = 0; i < 3; i++) {
          state.linear_vel[i] = sim->d->cvel[body_vel_adr + i];
          state.angular_vel[i] = sim->d->cvel[body_vel_adr + 3 + i];
        }
      }

      for (int i = 0; i < 3; i++) {
        state.size[i] = sim->m->geom_size[obj.geom_id * 3 + i];
      }
    }
  }

  /**
   * @brief Get the current state of an object
   * 
   * @param name Object name
   * @return const ObjectState* Pointer to object state, nullptr if not found
   */
  const ObjectState* get_object_state(const std::string& name) const {
    auto it = object_states.find(name);
    return it != object_states.end() ? &(it->second) : nullptr;
  }

  /**
   * @brief Get all object states
   * 
   * @return const std::unordered_map<std::string, ObjectState>& Map of all object states
   */
  const std::unordered_map<std::string, ObjectState>& get_all_object_states() const {
    return object_states;
  }

  /**
   * @brief Sample a goal state for a specific object and set it as the environment goal
   * 
   * @param object_name Name of the object to sample goal for
   * @param min_distance Minimum distance from current position
   * @param max_distance Maximum distance from current position
   * @return std::vector<double> Sampled goal state [x, y, z, qw, qx, qy, qz]
   */
  std::vector<double> set_goal_configuration(const std::string& object_name, 
                                                        double min_distance = 0.5, 
                                                        double max_distance = 2.0) {
    // Get object information
    auto object_info = get_object_info(object_name);
    if (!object_info) {
      throw std::runtime_error("Object not found: " + object_name);
    }
    
    // Sample a valid goal position
    std::array<double, 3> goal_pose;
    std::vector<double> random_state;
    double distance;
    
    // Keep sampling until we find a position within the desired distance range
    do {
      random_state = get_random_state();
      goal_pose = {random_state[0], random_state[1], 0.0};
      distance = std::sqrt(std::pow(object_info->position[0] - goal_pose[0], 2) + 
                           std::pow(object_info->position[1] - goal_pose[1], 2));
    } while (distance < min_distance || distance > max_distance);
    
    // Sample a random orientation
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> angle_dis(-M_PI, M_PI);
    double random_yaw = angle_dis(gen);
    std::array<double, 4> goal_quaternion = yaw_to_quaternion(random_yaw, true);
    
    // Set the goal in the environment
    MujocoGoal goal;
    goal.position = goal_pose;
    goal.orientation = goal_quaternion;
    goal.size = object_info->size;
    goal.geom_type = object_info->geom_type;
    set_goal(goal);
    
    // Return the full goal state
    std::vector<double> goal_state = {
      goal_pose[0], goal_pose[1], 0.0,
      goal_quaternion[0], goal_quaternion[1], goal_quaternion[2], goal_quaternion[3]
    };
    
    return goal_state;
  }

private:
  /**
   * @brief Warm up the simulator
   * 
   * Steps simulation a few times to stabilize physics
   */
  void warm_up()
  {
    for (int i = 0; i < 2; i++)
    {
      sim->step_simulation();
    }
  }

  /**
   * @brief Process and categorize objects in the environment
   * 
   * Identifies static and movable objects, stores their properties
   */
  void process_environment_objects() {
    static_objects.clear();
    movable_objects.clear();
    object_map.clear();

    // Iterate through all bodies in the environment
    for (int i = 0; i < sim->m->nbody; i++) {
      std::string body_name = std::string(sim->m->names + sim->m->name_bodyadr[i]);
      
      // Skip the robot and world bodies
      if (body_name == "robot" || body_name == "world") {
        continue;
      }

      // Get all geoms associated with this body
      for (int j = 0; j < sim->m->ngeom; j++) {

        if (sim->m->geom_bodyid[j] == i) {
          std::array<double, 3> size = {sim->m->geom_size[j * 3], sim->m->geom_size[j * 3 + 1], sim->m->geom_size[j * 3 + 2]};
          ObjectInfo obj(size);
          obj.body_id = i;
          obj.geom_id = j;
          std::string geom_name = std::string(sim->m->names + sim->m->name_geomadr[j]);
          obj.name = geom_name;
          // Cast int to mjtGeom enum
          obj.geom_type = static_cast<mjtGeom>(sim->m->geom_type[j]);
          // Store geometric properties
          for (int k = 0; k < 3; k++) {
            obj.position[k] = sim->m->geom_pos[j * 3 + k];
          }
          for (int k = 0; k < 4; k++) {
            obj.quaternion[k] = sim->m->geom_quat[j * 4 + k];
          }
          
          // Categorize objects based on name
          obj.is_static = (body_name.find("static") != std::string::npos || 
                          body_name.find("wall") != std::string::npos);

          // Store in appropriate vector and map
          if (obj.is_static) {
            static_objects.push_back(obj);
          } else if (body_name.find("movable") != std::string::npos) {
            movable_objects.push_back(obj);
          }
          object_map[geom_name] = obj;

          // Add collision pair with robot
          sim->add_pair(std::make_pair(body_name, "robot"));
        }
      }
    }

    std::cout << "Processed environment objects:\n";
    std::cout << "Static objects: " << static_objects.size() << "\n";
    std::cout << "Movable objects: " << movable_objects.size() << "\n";
  }

  // system properties
  std::shared_ptr<mujoco_simulator_t> sim;
  std::shared_ptr<system_group_t> sg;
  space_t* ss;
  space_t* cs;
  space_point_t current_qpos;

  // robot properties
  int robot_id;
  std::array<double, 3> init_robot_pos;

  // Object tracking
  std::vector<ObjectInfo> static_objects;
  std::vector<ObjectInfo> movable_objects;
  std::unordered_map<std::string, ObjectInfo> object_map;

  // Add robot info member
  ObjectInfo robot_info;

  // Object state tracking
  std::unordered_map<std::string, ObjectState> object_states;
};

}  // namespace prx 