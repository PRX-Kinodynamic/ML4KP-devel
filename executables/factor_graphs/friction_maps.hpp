#pragma once
#include "prx/utilities/defs.hpp"

using WorldFunction = std::function<void()>;
using namespace prx;

struct world_functions_t
{
  std::shared_ptr<system_group_t> _sg;
  space_t* ss;
  space_t* ps;
  world_functions_t(std::shared_ptr<system_group_t> sg) : _sg(sg)
  {
    ss = _sg->get_state_space();
    ps = _sg->get_parameter_space();
  }
  template <typename FrictionAt, typename... FrictionArgs>
  WorldFunction get_real_world(FrictionAt& friction_at, FrictionArgs... fargs)
  {
    std::function<void()> real_world = [&]()  // no-lint
    {
      const double x{ ss->at(0) };
      const double y{ ss->at(1) };
      auto friction_params = friction_at(x, y, fargs...);
      ps->copy_from(friction_params);
    };
    return real_world;
  }
};

namespace friction_maps
{
using FrictionVector = Eigen::Vector<double, 1>;

FrictionVector friction_map1_at(const double x, const double y)
{
  double friction = 1;
  const double max_friction{ 2 };
  if (y < 1.5)
  {
    friction = max_friction * y / 1.5;
  }
  else if (y < 1.7)
  {
    friction = max_friction;
  }
  else
  {
    friction = max_friction * (3 - y) / 1.3;
  }
  return FrictionVector(friction);
}

FrictionVector friction_map2_at(const double x, const double y)
{
  double friction = 1;
  const double max_friction{ 2 };
  if (y < 1.5)
  {
    friction = max_friction - max_friction * y / 1.5;
  }
  else if (y < 1.7)
  {
    friction = max_friction - max_friction;
  }
  else
  {
    friction = max_friction - max_friction * (3 - y) / 1.3;
  }
  return FrictionVector(friction);
}
FrictionVector friction_map3_at(const double x, const double y)
{
  return FrictionVector(1);
}

struct friction_maps_t
{
  // Asuming lower limit of all is (0,0)
  static inline std::unordered_map<int, double> x_max_map{ // no-lint
                                                           { 1, 10.0 },
                                                           { 2, 10.0 }
  };
  static inline std::unordered_map<int, double> y_max_map{ // no-lint
                                                           { 1, 10.0 },
                                                           { 2, 10.0 }
  };
  // static inline std::unordered_map<int, WorldFunction> world_functions_map{
  //   { 1, friction_maps::friction_map1_at },
  //   { 2, friction_maps::friction_map2_at }
  //   // no-lint
  // };
};

template <typename F, typename... Fargs>
void friction_map_to_file(const int map_id, const double stepping, F& f, Fargs... fargs)
{
  const std::string fm_out_dir = prx::out_path + "friction_maps/";
  logger_t logger(fm_out_dir + "friction_map_" + std::to_string(map_id) + ".txt", ' ');
  const double x_max{ friction_maps_t::x_max_map[map_id] };
  const double y_max{ friction_maps_t::y_max_map[map_id] };
  for (double x = 0; x < x_max; x += stepping)
  {
    for (double y = 0; y < y_max; y += stepping)
    {
      const FrictionVector friction_params{ f(x, y, fargs...) };
      logger.log(x, y, friction_params.transpose());
    }
  }
}
}  // namespace friction_maps

template <typename Plant>
void mecanum_omnibot_follow_path(Plant& plant, std::shared_ptr<system_group_t> sg,
                                 const trajectory_t& desired_trajectory, trajectory_t& resulting_trajectory,
                                 plan_t& resulting_plan)
{
  const double l_a = .11;
  const double l_b = .10;
  const double l_ab = l_a + l_b;
  const auto ss = sg->get_state_space();
  const auto cs = sg->get_control_space();
  const auto ps = sg->get_parameter_space();
  Eigen::Matrix<double, 4, 3> inverse;
  inverse << -1.0 / (4.0 * l_ab), 1.0 / (4.0 * l_ab), 1.0 / 4.0,  // no-lint
      1.0 / (4.0 * l_ab), 1.0 / (4.0 * l_ab), -1.0 / 4.0,         // no-lint
      -1.0 / (4.0 * l_ab), 1.0 / (4.0 * l_ab), -1.0 / 4.0,        // no-lint
      1.0 / (4.0 * l_ab), 1.0 / (4.0 * l_ab), 1.0 / 4.0;          // no-lint
  double curr_freq{ 0.0 };
  Eigen::Vector3d current_state_vec;
  std::shared_ptr<custom_controller_t> omnibot_controller =
      std::make_shared<custom_controller_t>(plant, "omnibot_controller");

  double goal_region_radius{ 0.01 };
  Eigen::Vector4d U;
  bool controller_reached_goal{ true };
  omnibot_controller->custom_control_function = [&](const space_point_t& goal, const space_point_t& control) {
    ss->copy_to(current_state_vec);
    const Eigen::Vector3d xd{ goal->vector() - current_state_vec };
    // const Eigen::Vector3d xd{ current_state_vec - goal->vector() };
    // PRX_DEBUG_VAR_1(current_state_vec.transpose());
    // PRX_DEBUG_VAR_1(xd.transpose());
    // PRX_DEBUG_VAR_2(curr_freq, xd.norm());
    if (xd.norm() < goal_region_radius)
    {
      U = Eigen::Vector4d::Zero();
      curr_freq = 0.0;
      controller_reached_goal = true;
    }
    else if (curr_freq <= 0.0)
    {
      U = (inverse * xd).normalized() * 128;
      curr_freq = 0.12;
    }
    cs->copy(control, U);
    curr_freq -= simulation_step;
  };
  space_point_t goal = ss->make_point();
  condition_check_t checker("time", 1);

  // auto cc = create_default_goal_check(ss, goal, goal_region_radius);
  custom_check_t goal_reached = [&]() { return controller_reached_goal; };
  condition_check_t check_goal_reached(goal_reached);
  checker.add_condition(&check_goal_reached);

  space_point_t start_state = ss->make_point();
  space_point_t end_state = ss->make_point();
  trajectory_t local_traj(ss);

  start_state = desired_trajectory[0];
  for (int i = 0; i < desired_trajectory.size() - 1; ++i)
  {
    goal = desired_trajectory[i + 1];
    omnibot_controller->set_goal(goal);
    checker.reset();
    sg->propagate(start_state, omnibot_controller, checker, local_traj);
    resulting_trajectory += local_traj;
    resulting_plan += *(omnibot_controller->get_plan());
    omnibot_controller->get_plan()->clear();
    controller_reached_goal = false;
    start_state = resulting_trajectory.back();
  }
}