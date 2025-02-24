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

    init_robot_pos = &sim->d->geom_xpos[3 * robot_id];
    current_state = ss->make_point();

  }

  std::vector<double> get_random_state() {
    // returns a random x, y, yaw in the environment
    std::vector<double> bounds = get_environment_bounds();
    std::vector<double> state(3);
    state[0] = uniform_random(bounds[0], bounds[1]);
    state[1] = uniform_random(bounds[2], bounds[3]);
    state[2] = uniform_random(-M_PI, M_PI);
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
    ss->copy_to(current_state);
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

    ss->copy_to(current_state);

    sg->propagate(current_state, plan, traj);
    
    ss->copy_from(traj.back());
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
  space_point_t get_current_state()
  {
    return current_state;
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
          ObjectInfo obj;
          obj.body_id = i;
          obj.geom_id = j;
          std::string geom_name = std::string(sim->m->names + sim->m->name_geomadr[j]);
          obj.name = geom_name;
          
          // Store geometric properties
          for (int k = 0; k < 3; k++) {
            obj.position[k] = sim->m->geom_pos[j * 3 + k];
            obj.size[k] = sim->m->geom_size[j * 3 + k];
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
  space_point_t current_state;

  // robot properties
  int robot_id;
  double* init_robot_pos;

  // Object tracking
  std::vector<ObjectInfo> static_objects;
  std::vector<ObjectInfo> movable_objects;
  std::unordered_map<std::string, ObjectInfo> object_map;

  // Add robot info member
  ObjectInfo robot_info;
};

}  // namespace prx 