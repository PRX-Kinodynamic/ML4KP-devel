#include "prx/utilities/defs.hpp"
#include "prx/mujoco/mj_simulator.hpp"
#include <vector>
#include <memory>
#include <iostream>
#include <nlohmann/json.hpp>

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
    x_min_ = x_min + 2 * cylinder_radius;
    x_max_ = x_max - 2 * cylinder_radius;
    y_min_ = y_min + 2 * cylinder_radius;
    y_max_ = y_max - 2 * cylinder_radius;

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
  PushEnvironment(const std::string& xml_path, bool visualize)
    : sim(std::make_shared<mujoco_simulator_t>(xml_path, visualize))
  {
    sim->init_simulator();
    warm_up();

    auto context = sim->get_context("mujoco");
    sg = context.first;
    ss = sg->get_state_space();
    cs = sg->get_control_space();

    robot_id = mj_name2id(sim->m, mjOBJ_GEOM, "robot");
    cylinder_id = mj_name2id(sim->m, mjOBJ_GEOM, "obstacle_movable_1");

    

    // Store cylinder radius from the model
    cylinder_radius = sim->m->geom_size[cylinder_id * 3];  // First element of size array is radius for cylinder
    robot_radius = sim->m->geom_size[robot_id * 3];        // For sphere, size is radius


    // look for a body named walls in the model and find the bounds of the environment by parsing each geom element and
    // finding the min and max x and y values
    auto walls = mj_name2id(sim->m, mjOBJ_BODY, "walls");
    auto geom = sim->m->body_geomadr[walls];
    double x_min = std::numeric_limits<double>::infinity();
    double x_max = -std::numeric_limits<double>::infinity();
    double y_min = std::numeric_limits<double>::infinity();
    double y_max = -std::numeric_limits<double>::infinity();
    for (int i = 0; i < sim->m->ngeom; i++)
    {
      if (sim->m->geom_bodyid[i] == walls)
      {
        x_min = std::min(x_min, sim->m->geom_pos[i * 3]);
        x_max = std::max(x_max, sim->m->geom_pos[i * 3]);
        y_min = std::min(y_min, sim->m->geom_pos[i * 3 + 1]);
        y_max = std::max(y_max, sim->m->geom_pos[i * 3 + 1]);
      }
    }

    set_bounds(x_min, x_max, y_min, y_max);

    location_iterator = std::make_shared<CylinderLocationIterator>(ss, ss->get_lower_bounds()[0],
                                                                   ss->get_upper_bounds()[0], ss->get_lower_bounds()[1],
                                                                   ss->get_upper_bounds()[1], cylinder_radius,
                                                                   0.1  // default discretization step
    );

    init_robot_pos = &sim->d->geom_xpos[3 * robot_id];

    // Create reusable state points
    current_state = ss->make_point();
    cylinder_state = ss->make_point();

    // return_state = ss->make_point();
  }


  void add_pair()
  {
    for (int i = 0; i < sim->m->ngeom; i++)
    {
      std::string geom_name = std::string(sim->m->names + sim->m->name_geomadr[i]);
      if (geom_name == "obstacle_movable_1" || geom_name == "robot")
      {
        continue;
      }
      else
      {
        // check if name contains "wall" or "static"
        if (geom_name.find("static") != std::string::npos)
        {
          sim->add_pair(std::make_pair(geom_name, "obstacle_movable_1"));
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
    setup_robot_position(goal_state);

    std::vector<double> goal_state_vec;
    ss->copy_vector_from_point(goal_state_vec, goal_state);
    sim->set_goal(goal_state_vec);
    sim->set_goal_radius(0.3);
    warm_up();
  }

  void step(const space_point_t& control, double duration)
  {
    for (int i = 0; i < sim->m->nu; i++)
    {
      sim->d->qvel[i] = 0.0;
    }
    sim->step_simulation();

    trajectory_t traj(ss);
    plan_t plan(cs);

    plan.append_onto_back(duration);
    plan.back().control = control;

    ss->copy_to(current_state);

    sg->propagate(current_state, plan, traj);
    ss->copy_from(traj.back());
  }

  space_point_t get_cylinder_state()
  {
    // return the geom xpos of the cylinder
    double* cylinder_pos = &sim->d->geom_xpos[3 * cylinder_id];
    cylinder_state->at(0) = cylinder_pos[0];
    cylinder_state->at(1) = cylinder_pos[1];
    return cylinder_state;
  }

  bool is_in_goal_region(const space_point_t& goal_state)
  {
    double* cylinder_pos = &sim->d->geom_xpos[3 * cylinder_id];
    return (cylinder_pos[0] - goal_state->at(0)) * (cylinder_pos[0] - goal_state->at(0)) +
               (cylinder_pos[1] - goal_state->at(1)) * (cylinder_pos[1] - goal_state->at(1)) <
           0.01;
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

  // Returns nullptr when all goals have been exhausted
  space_point_t next_goal()
  {
    if (!location_iterator->has_next())
    {
      return nullptr;
    }

    auto goal_state = location_iterator->next();

    reset(goal_state);
    return goal_state;
  }

  // Reset iterator to start over with goals
  void reset_goals()
  {
    location_iterator->reset();
  }

  // Set discretization step size (will reset iterator)
  void set_discretization(double step)
  {
    location_iterator = std::make_shared<CylinderLocationIterator>(ss, ss->get_lower_bounds()[0],
                                                                   ss->get_upper_bounds()[0], ss->get_lower_bounds()[1],
                                                                   ss->get_upper_bounds()[1], cylinder_radius, step);
  }

  int get_total_goals() const
  {
    return location_iterator->get_total_goals();
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

private:
  void warm_up()
  {
    for (int i = 0; i < 2; i++)
    {
      sim->step_simulation();
    }
  }

  void setup_robot_position(const space_point_t& goal_state)
  {
    sim->reset_simulation();
    double* cylinder_pos = &sim->d->geom_xpos[3 * cylinder_id];
    // double* init_robot_pos = &sim->d->geom_xpos[3*robot_id];

    double dx = goal_state->at(0) - cylinder_pos[0];
    double dy = goal_state->at(1) - cylinder_pos[1];
    double angle = atan2(dy, dx);

    // Now using the stored radius values
    double total_radius = cylinder_radius + robot_radius;

    double robot_x = cylinder_pos[0] - total_radius * cos(angle);
    double robot_y = cylinder_pos[1] - total_radius * sin(angle);

    sim->d->qpos[0] = robot_x - init_robot_pos[0];
    sim->d->qpos[1] = robot_y - init_robot_pos[1];
    sim->d->qvel[0] = 0.0;
    sim->d->qvel[1] = 0.0;
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

  // cylinder properties
  int cylinder_id;
  double cylinder_radius;
  space_point_t cylinder_state;  // this is the position of the cylinder relative to the origin of the environment

  std::shared_ptr<CylinderLocationIterator> location_iterator;  // goal iterator
};
}  // namespace prx