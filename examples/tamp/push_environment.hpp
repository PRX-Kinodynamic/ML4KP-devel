#include "prx/utilities/defs.hpp"
#include "prx/mujoco/mj_simulator.hpp"
#include <vector>
#include <memory>
#include <iostream>
#include <nlohmann/json.hpp>
#include <random>
#include <fstream>
#include <sstream>

namespace prx
{

using json = nlohmann::json; 
// Internal helper class - now private to PushEnvironment
class CylinderLocationIterator
{
public:
  CylinderLocationIterator(space_t* state_space, double x_min, double x_max, double y_min, double y_max,
                           double cylinder_radius, double discretization_step)
    : ss(state_space), step(discretization_step)
  {
    // Account for cylinder radius in bounds
    x_min_ = x_min + 2 * cylinder_radius; // -1.6
    x_max_ = x_max - 2 * cylinder_radius; // 1.6
    y_min_ = y_min + 2 * cylinder_radius; // -1.6
    y_max_ = y_max - 2 * cylinder_radius; // 1.6

    // Initialize current position
    current_x = x_min_;
    current_y = y_min_;

    // Calculate number of points in each dimension
    nx = static_cast<int>((x_max_ - x_min_) / step) + 1;
    ny = static_cast<int>((y_max_ - y_min_) / step) + 1;

    // Initialize indices
    ix = 0;
    iy = 0;

    done = false;

    // Create state point to reuse
    current_point = ss->make_point();
  }

  bool has_next() const
  {
    return !done;
  }

  space_point_t next()
  {
    if (done)
    {
      throw std::runtime_error("Iterator is done");
    }

    // Update state point
    current_point->at(0) = current_x;
    current_point->at(1) = current_y;

    // Update position for next call
    ix++;
    current_x = x_min_ + ix * step;

    if (current_x > x_max_)
    {
      ix = 0;
      current_x = x_min_;
      iy++;
      current_y = y_min_ + iy * step;

      if (current_y > y_max_)
      {
        done = true;
      }
    }

    return current_point;
  }

  int get_total_goals() const
  {
    return nx * ny;
  }

  void reset()
  {
    current_x = x_min_;
    current_y = y_min_;
    ix = 0;
    iy = 0;
    done = false;
  }

private:
  space_t* ss;
  space_point_t current_point;
  double x_min_, x_max_, y_min_, y_max_;
  double current_x, current_y;
  double step;
  int ix, iy;
  int nx, ny;
  bool done;
};

class PushEnvironment
{
public:
  enum class GoalSamplingMode {
    DISCRETIZED,
    RANDOM,
    FILE
  };

  double goal_radius;

  PushEnvironment(const std::string& xml_path, bool visualize, double goal_radius)
    : sim(std::make_shared<mujoco_simulator_t>(xml_path, visualize))
  {
    sim->init_simulator();
    warm_up();

    auto context = sim->get_context("mujoco");
    sg = context.first;
    ss = sg->get_state_space();
    cs = sg->get_control_space();

    this->goal_radius = goal_radius;

    // Update to only track robot (which is now the cylinder)
    robot_id = mj_name2id(sim->m, mjOBJ_GEOM, "robot");
    robot_radius = sim->m->geom_size[robot_id * 3];  // First element of size array is radius for cylinder

    // look for a body named walls in the model and find the bounds of the environment by parsing each geom element and
    // finding the min and max x and y values
    auto walls = mj_name2id(sim->m, mjOBJ_BODY, "walls");
    auto geom = sim->m->body_geomadr[walls];
    double x_min = std::numeric_limits<double>::infinity();
    double x_max = -std::numeric_limits<double>::infinity();
    double y_min = std::numeric_limits<double>::infinity();
    double y_max = -std::numeric_limits<double>::infinity();

    // for (int i = 0; i < sim->m->ngeom; i++)
    // {
    //   if (sim->m->geom_bodyid[i] == walls)
    //   {
    //     x_min = std::min(x_min, *(sim->m->geom_pos + i * 3));
    //     x_max = std::max(x_max, *(sim->m->geom_pos + i * 3));
    //     y_min = std::min(y_min, *(sim->m->geom_pos + i * 3 + 1));
    //     y_max = std::max(y_max, *(sim->m->geom_pos + i * 3 + 1));
    //   }
    // }

    // artificially set bounds to be 2 units away from the origin

    
    x_min = -2.0; // std::min(x_min, -2);
    x_max = 2.0; // std::max(x_max, 2);
    y_min = -2.0; // std::min(y_min, -2);
    y_max = 2.0; // std::max(y_max, 2);

    std::cout << "x_min: " << x_min << ", x_max: " << x_max << ", y_min: " << y_min << ", y_max: " << y_max << std::endl;
    set_bounds(x_min, x_max, y_min, y_max);

    location_iterator = std::make_shared<CylinderLocationIterator>(ss, ss->get_lower_bounds()[0],
                                                                   ss->get_upper_bounds()[0], ss->get_lower_bounds()[1],
                                                                   ss->get_upper_bounds()[1], robot_radius,
                                                                   0.1  // default discretization step
    );

    init_robot_pos = &sim->d->geom_xpos[3 * robot_id];

    // std::cout << "init_robot_pos: " << init_robot_pos[0] << ", " << init_robot_pos[1] << std::endl;

    // Create reusable state points
    current_state = ss->make_point();
  }

  void add_pair()
  {

    // std::cout << "N"
    for (int i = 0; i < sim->m->nbody; i++)
    {
      
      std::string body_name = std::string(sim->m->names + sim->m->name_bodyadr[i]);
      if (body_name == "robot")
      {
        continue;
      }
      else
      {
        // check if name contains "wall" or "obstacle"
        if (body_name.find("wall") != std::string::npos || body_name.find("obstacle") != std::string::npos)
        {
          sim->add_pair(std::make_pair(body_name, "robot"));
        }
      }
    }
  }

  void set_bounds(double x_min, double x_max, double y_min, double y_max)
  {
    std::vector<double> ub = ss->get_upper_bounds();
    std::vector<double> lb = ss->get_lower_bounds();

    lb.at(0) = x_min;
    lb.at(1) = y_min;
    ub.at(0) = x_max;
    ub.at(1) = y_max;

    ss->set_bounds(lb, ub);
  }

  void reset(const space_point_t& goal_state)
  {
    sim->reset_simulation();
    warm_up();

    std::vector<double> goal_state_vec;
    ss->copy_vector_from_point(goal_state_vec, goal_state);
    sim->set_goal(goal_state_vec);
    sim->set_goal_radius(goal_radius);
    
    ss->copy_to(current_state);

  }

  void step(const space_point_t& control, double duration)
  {
  //   for (int i = 0; i < sim->m->nu; i++)
  //   {
  //     sim->d->qvel[i] = 0.0;
  //   }

  //   sim->step_simulation();

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

  bool is_in_goal_region(const space_point_t& goal_state)
  {
    double* robot_pos = &sim->d->geom_xpos[3 * robot_id];
    return sqrt((robot_pos[0] - goal_state->at(0)) * (robot_pos[0] - goal_state->at(0)) +
           (robot_pos[1] - goal_state->at(1)) * (robot_pos[1] - goal_state->at(1))) <
           goal_radius;
  }

  bool is_in_collision()
  {
    return sim->in_collision();
  }

  space_point_t get_state_space_point()
  {
    return ss->make_point();
  }
  space_point_t get_control_space_point()
  {
    return cs->make_point();
  }

  void set_discretization(double step) {
    location_iterator = std::make_shared<CylinderLocationIterator>(ss, ss->get_lower_bounds()[0], ss->get_upper_bounds()[0], ss->get_lower_bounds()[1], ss->get_upper_bounds()[1], robot_radius, step);
  }

  // Set the goal sampling mode
  void set_goal_sampling_mode(GoalSamplingMode mode) {
    sampling_mode = mode;
    reset_goals();
  }

  // Set number of random goals to generate
  void set_random_goals(int num_goals) {
    random_goals.clear();
    random_goals.reserve(num_goals);
    
    // Get bounds accounting for cylinder radius
    double x_min = ss->get_lower_bounds()[0] + 2 * robot_radius;
    double x_max = ss->get_upper_bounds()[0] - 2 * robot_radius;
    double y_min = ss->get_lower_bounds()[1] + 2 * robot_radius;
    double y_max = ss->get_upper_bounds()[1] - 2 * robot_radius;

    // Create random distribution
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> x_dist(x_min, x_max);
    std::uniform_real_distribution<> y_dist(y_min, y_max);

    // Generate random goals
    for (int i = 0; i < num_goals; i++) {
      space_point_t goal = ss->make_point();
      goal->at(0) = x_dist(gen);
      goal->at(1) = y_dist(gen);
      random_goals.push_back(goal);
    }
    current_goal_index = 0;
  }

  // Add new method to load goals from file
  void load_goals_from_file(const std::string& filename) {
    random_goals.clear();  // We'll reuse the random_goals vector for file goals
    std::ifstream file(filename);
    
    if (!file.is_open()) {
      throw std::runtime_error("Could not open goal file: " + filename);
    }

    std::string line;
    while (std::getline(file, line)) {
      // Skip empty lines
      if (line.empty()) continue;

      // Parse x,y coordinates
      std::stringstream line_stream(line);  // Renamed from ss to line_stream
      std::string x_str, y_str;
      
      if (std::getline(line_stream, x_str, ',') && std::getline(line_stream, y_str)) {
        try {
          double x = std::stod(x_str);
          double y = std::stod(y_str);
          
          // Create goal state point
          space_point_t goal = ss->make_point();  // Now ss refers to the state space
          goal->at(0) = x;
          goal->at(1) = y;
          random_goals.push_back(goal);
        } catch (const std::exception& e) {
          std::cerr << "Error parsing line: " << line << std::endl;
          continue;
        }
      }
    }
    
    current_goal_index = 0;
    file.close();
  }

  // Modified next_goal to handle file mode
  space_point_t next_goal() {
    if (sampling_mode == GoalSamplingMode::DISCRETIZED) {
      if (!location_iterator->has_next()) {
        return nullptr;
      }
      auto goal_state = location_iterator->next();
      reset(goal_state);
      return goal_state;
    } else {  // RANDOM or FILE mode
      if (random_goals.empty() || current_goal_index >= random_goals.size()) {
        return nullptr;
      }
      auto goal_state = random_goals[current_goal_index++];
      reset(goal_state);
      return goal_state;
    }
  }

  // Modified reset_goals to handle file mode
  void reset_goals() {
    if (sampling_mode == GoalSamplingMode::DISCRETIZED) {
      location_iterator->reset();
    } else {  // RANDOM or FILE mode
      current_goal_index = 0;
    }
  }

  // Modified get_total_goals to handle file mode
  int get_total_goals() const {
    if (sampling_mode == GoalSamplingMode::DISCRETIZED) {
      return location_iterator->get_total_goals();
    } else {  // RANDOM or FILE mode
      return random_goals.size();
    }
  }

  space_point_t get_current_state()
  {
    // this is the [qpos, qvel] state, "ss->copy_to(ss->make_point())" is the state space point
    return current_state;
  }

  space_t* get_state_space() const
  {
    return ss;
  }

  space_t* get_control_space() const
  {
    return cs;
  }

  // Add to PushEnvironment class declaration
  json get_geom_info() const
  {
    json geom_info;
    for (int i = 0; i < sim->m->ngeom; i++)
    {
      json geom;
      geom["name"] = std::string(sim->m->names + sim->m->name_geomadr[i]);
      geom["pos"] = { sim->m->geom_pos[i * 3], sim->m->geom_pos[i * 3 + 1], sim->m->geom_pos[i * 3 + 2] };
      geom["size"] = { sim->m->geom_size[i * 3], sim->m->geom_size[i * 3 + 1], sim->m->geom_size[i * 3 + 2] };
      geom["quat"] = { sim->m->geom_quat[i * 4], sim->m->geom_quat[i * 4 + 1], sim->m->geom_quat[i * 4 + 2],
                       sim->m->geom_quat[i * 4 + 3] };
      geom_info.push_back(geom);
    }
    return geom_info;
  }

  space_point_t get_goal_at_index(int index) {
    if (index >= get_total_goals()) {
        return nullptr;
    }
    // Return the goal at the specified index
    return location_iterator->next();
  }

private:
  void warm_up()
  {
    for (int i = 0; i < 2; i++)
    {
      sim->step_simulation();
    }
  }

  // system properties
  std::shared_ptr<mujoco_simulator_t> sim;
  std::shared_ptr<system_group_t> sg;
  space_t* ss;
  space_t* cs;
  space_point_t current_state;  // this is the [qpos, qvel] state, "ss->copy_to(ss->make_point())" is the state space
                                // point

  // robot properties
  int robot_id;
  double* init_robot_pos;  // this is the initial position of the robot relative to the origin of the environment, used
                           // to offset the robot qpos for establishing pushing point on the cylinder
  double robot_radius;

  std::shared_ptr<CylinderLocationIterator> location_iterator;  // goal iterator

  GoalSamplingMode sampling_mode = GoalSamplingMode::DISCRETIZED;
  std::vector<space_point_t> random_goals;
  size_t current_goal_index = 0;
};
}  // namespace prx