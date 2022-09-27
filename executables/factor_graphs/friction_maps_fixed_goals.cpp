#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>
#include <boost/algorithm/string/join.hpp>
#include <boost/optional.hpp>

#include "prx/planning/loaders/planner_loader.hpp"
#include "prx/planning/planners/rrt.hpp"
#include "prx/planning/planner_statistics.hpp"
#include "prx/planning/world_model.hpp"

#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/simulation/playback/trajectory.hpp"
#include "prx/simulation/system_factory.hpp"
#include "prx/simulation/system_group.hpp"

#include "prx/utilities/defs.hpp"

#include "prx/visualization/three_js_group.hpp"

#include "prx/gtdynamics/defs.hpp"
#include "prx/gtdynamics/utilities/fg_logger.hpp"
#include "prx/gtdynamics/planning/trajectory_fg.hpp"
#include "prx/gtdynamics/planning/trajectory_optimizer.hpp"
#include "prx/gtdynamics/utilities/utilities_functions.hpp"
#include "prx/gtdynamics/planning/initialization_trajs_fg.hpp"
#include "prx/simulation/controllers/custom_controller.hpp"

namespace fs = std::filesystem;
using namespace prx;

int main(int argc, char* argv[])
{
  auto params = param_loader("executables/factor_graphs/friction_map_random_goals.yaml", argc, argv);

  simulation_step = params["simulation_step"].as<double>();
  init_random(params["random_seed"].as<int>());

  std::string plant_name = params["/plant/name"].as<>();
  std::string plant_path = params["/plant/path"].as<>();

  auto obstacles = load_obstacles(params["environment"].as<>());

  auto obstacle_list = obstacles.second;
  auto obstacle_names = obstacles.first;

  auto plant = system_factory_t::create_system(plant_name, plant_path);
  world_model_t world_model({ plant }, {});
  world_model.create_context("context", { plant_name }, {});
  PRX_DEBUG_PRINT

  auto context = world_model.get_context("context");
  auto sg = context.first;
  const auto ss = sg->get_state_space();
  const auto cs = sg->get_control_space();
  const auto ps = sg->get_parameter_space();
  const auto ss_dim = ss->get_dimension();
  const auto cs_dim = cs->get_dimension();
  const auto ps_dim = ps->get_dimension();
  prx_assert(ps != nullptr, "Parameter space is null!!!");

  ss->set_bounds({ 0.0, 0.0, -M_PI }, { 10.0, 10.0, M_PI });

  auto cs_lb = params["/plant/control_space_lower_bound"].as<std::vector<double>>();
  auto cs_up = params["/plant/control_space_upper_bound"].as<std::vector<double>>();
  cs->set_bounds(cs_lb, cs_up);

  std::ofstream ofs_trajs, ofs_frmap, ofs_plans, ofs_goals, ofs_traj_real;
  ofs_trajs.open("mecanum_analytical_fixed_goals_trajs_predicted.txt", std::ofstream::trunc);
  ofs_traj_real.open("mecanum_analytical_fixed_goals_trajs_real.txt", std::ofstream::trunc);
  ofs_goals.open("mecanum_analytical_fixed_goals.txt", std::ofstream::trunc);
  // ofs_plans.open("mecanum_analytical_plan.txt", std::ofstream::trunc);
  // ofs_frmap.open("mecanum_friction_map_gt.txt", std::ofstream::trunc);
  ofs_frmap.open("mecanum_friction_map_fixed_goals.txt", std::ofstream::trunc);
  auto friction_map_gt = [&](const double x, const double y) {
    const double friction{ x / 10.0 + y / 10.0 };
    return friction;
  };

  bool write_to_file = false;
  world_model.world_change_function = [&]() {
    if (write_to_file)
    {
      const double x{ ss->at(0) };
      const double y{ ss->at(1) };
      const double th{ ss->at(2) };
      const double l_a{ 0.11 };
      const double l_b{ 0.10 };
      const double x1{ x + l_a * std::cos(th) - l_b * std::sin(th) };
      const double y1{ y + l_a * std::sin(th) + l_b * std::cos(th) };
      const double x2{ x + (-l_a) * std::cos(th) - l_b * std::sin(th) };
      const double y2{ y + (-l_a) * std::sin(th) + l_b * std::cos(th) };
      const double x3{ x + l_a * std::cos(th) - (-l_b) * std::sin(th) };
      const double y3{ y + l_a * std::sin(th) + (-l_b) * std::cos(th) };
      const double x4{ x + (-l_a) * std::cos(th) - (-l_b) * std::sin(th) };
      const double y4{ y + (-l_a) * std::sin(th) + (-l_b) * std::cos(th) };
      Eigen::Vector4d friction_params{ friction_map_gt(x1, y1), friction_map_gt(x2, y2), friction_map_gt(x3, y3),
                                       friction_map_gt(x4, y4) };
      ps->copy_from_vector(friction_params);

      ofs_frmap << x1 << " " << y1 << " " << friction_params[0] << std::endl;
      ofs_frmap << x2 << " " << y2 << " " << friction_params[1] << std::endl;
      ofs_frmap << x3 << " " << y3 << " " << friction_params[2] << std::endl;
      ofs_frmap << x4 << " " << y4 << " " << friction_params[3] << std::endl;
    }
    else
    {
      ps->copy_from_vector({ 1, 1, 1, 1 });
    }
  };

  std::shared_ptr<custom_controller_t> controller = std::make_shared<custom_controller_t>(plant, "custom_ctrl");
  controller->custom_control_function = [](const space_point_t& goal, space_point_t& control) {

  };

  condition_check_t checker(params["checker_type"].as<>(), params["checker_value"].as<int>());

  const int num_trajs{ params["num_trajs"].as<int>() };
  std::vector<std::vector<double>> goals = { { 1, 1, 0 }, { 9, 9, 0 }, { 9, 1, 0 },
                                             { 1, 9, 0 }, { 5, 1, 0 }, { 5, 9, 0 } };
  for (int i = 0; i < goals.size(); ++i)
  {
    ofs_goals << i << " " << goals[i][0] << " " << goals[i][1] << " " << goals[i][2] << std::endl;
  }
  ss->copy_from_vector(goals[0]);

  ss->copy_to_point(rrt_query.start_state);

  trajectory_t traj_real(ss);
  for (int i = 0; i < num_trajs; ++i)
  {
    ss->copy_point_from_vector(rrt_query.goal_state, goals[(i + 1) % goals.size()]);

    write_to_file = true;

    for (auto state : rrt_query.solution_traj)
    {
      ofs_trajs << simulation_step << " " << state << " " << i << "\n";
      // ss->copy_from_point(state);
      // world_model.world_change_function();
    }
    sg->propagate(rrt_query.start_state, rrt_query.solution_plan, traj_real);
    for (auto state : traj_real)
    {
      ofs_traj_real << simulation_step << " " << state << " " << i << "\n";
    }
    write_to_file = false;

    ss->copy_point(rrt_query.start_state, rrt_query.solution_traj.back());

    ofs_trajs << "\n";
    checker.reset();
  }
}