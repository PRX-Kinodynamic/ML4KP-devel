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

#include "prx/factor_graphs/defs.hpp"
#include "prx/factor_graphs/utilities/fg_logger.hpp"
#include "prx/factor_graphs/planning/trajectory_fg.hpp"
#include "prx/factor_graphs/planning/trajectory_optimizer.hpp"
#include "prx/factor_graphs/utilities/utilities_functions.hpp"
#include "prx/factor_graphs/planning/initialization_trajs_fg.hpp"

#include "prx/simulation/controllers/custom_controller.hpp"
#include "prx/simulation/plants/types/noisy_plant.hpp"

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

  auto system = system_factory_t::create_system(plant_name, plant_path);
  auto plant = std::dynamic_pointer_cast<plant_t>(system);
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

  PRX_DEBUG_PRINT
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
    // const double friction{ x / 10.0 + y / 10.0 };
    double friction = 1;
    if (y < 4)
    {
      friction = 2 * y / 4;
    }
    else if (y < 6)
    {
      friction = 2;
    }
    else
    {
      friction = 2 * (10 - y) / 4;
    }
    return friction;
  };

  PRX_DEBUG_PRINT
  bool write_to_file = false;
  world_model.world_change_function = [&]() {
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
    ps->copy_from(friction_params);

    if (write_to_file)
    {
      ofs_frmap << x1 << " " << y1 << " " << friction_params[0] << std::endl;
      ofs_frmap << x2 << " " << y2 << " " << friction_params[1] << std::endl;
      ofs_frmap << x3 << " " << y3 << " " << friction_params[2] << std::endl;
      ofs_frmap << x4 << " " << y4 << " " << friction_params[3] << std::endl;
    }
    // else
    // {
    //   ps->copy_from({ 1, 1, 1, 1 });
    // }
  };

  // for (double i = 0; i < 10; i += 0.01)
  // {
  //   for (double j = 0; j < 10; j += 0.01)
  //   {
  //     ss->copy_from(std::vector{ i, j, 0.0 });
  //     world_model.world_change_function();
  //   }
  // }
  // return 0;
  condition_check_t checker(params["checker_type"].as<>(), params["checker_value"].as<int>());

  std::vector<std::vector<double>> goals = { { 1, 1, 0 }, { 9, 9, 0 }, { 9, 1, 0 },
                                             { 1, 9, 0 }, { 5, 1, 0 }, { 5, 9, 0 } };
  for (int i = 0; i < goals.size(); ++i)
  {
    ofs_goals << i << " " << goals[i][0] << " " << goals[i][1] << " " << goals[i][2] << std::endl;
  }

  space_point_t start_state = ss->make_point();
  ss->copy_point_from_vector(start_state, goals[0]);

  std::shared_ptr<custom_controller_t> omnibot_controller =
      std::make_shared<custom_controller_t>(plant, "omnibot_controller");

  const double l_a = .11;
  const double l_b = .10;
  const double l_ab = l_a + l_b;

  PRX_DEBUG_PRINT
  Eigen::MatrixXd inverse(4, 3);
  inverse << -1.0 / (4.0 * l_ab), 1.0 / (4.0 * l_ab), 1.0 / 4.0,  // no-lint
      1.0 / (4.0 * l_ab), 1.0 / (4.0 * l_ab), -1.0 / 4.0,         // no-lint
      -1.0 / (4.0 * l_ab), 1.0 / (4.0 * l_ab), -1.0 / 4.0,        // no-lint
      1.0 / (4.0 * l_ab), 1.0 / (4.0 * l_ab), 1.0 / 4.0;          // no-lint

  Eigen::Vector3d current_state_vec;
  const double frequency{ params["frequency"].as<double>() };
  double curr_freq{ 0 };

  auto n_plant = new prx::noisy_plant_t<prx::gaussian_noise_t>(plant, 0, 0.01);
  auto n_ss = n_plant->get_state_space();

  const bool noisy_plant{ params["noisy_plant"].as<bool>() };
  const space_t* ss_curr;
  if (noisy_plant)
  {
    ss_curr = n_ss;
  }
  else
  {
    ss_curr = ss;
  }
  Eigen::Vector4d U{ Eigen::Vector4d::Zero() };
  // clang-format off
  omnibot_controller -> custom_control_function = [&](const space_point_t& goal, const space_point_t& control) 
  {
    curr_freq += simulation_step;
    ss_curr -> copy_to(current_state_vec);
    const Eigen::Vector3d xd{ goal->vector<>() - current_state_vec};
    if (xd.norm() < 0.25)
    {
      // plan.append_onto_back(simulation_step);
      // cs->copy(plan.back().control, Eigen::Vector4d::Zero());
      U =  Eigen::Vector4d::Zero();
    }
    else if (curr_freq >= frequency)
    {
      U = (inverse * xd).normalized() * 128;
    // std::cout << "goal: " << goal->vector<>().transpose() << std::endl;
    // std::cout << "x: " << current_state_vec.transpose() << "\t|xd|: " << xd.norm() << std::endl;
    // std::cout << "condition: " << checker << std::endl;
    //   std::cout << "U: " << U.transpose() << std::endl;
      // cs -> copy(control, U);
      // plan.append_onto_back(simulation_step);
      // cs->copy(plan.back().control, U);
      curr_freq = 0;
    }

    cs -> copy(control, U);

    // std::cout << "x: " << x << "\t"
  };
  // clang-format on
  trajectory_t traj_real(ss);
  trajectory_t accum_traj(ss);
  const int num_trajs{ params["num_trajs"].as<int>() };

  PRX_DEBUG_PRINT
  for (int i = 0; i < num_trajs; ++i)
  {
    curr_freq = 0;
    traj_real.clear();
    checker.reset();
    omnibot_controller->get_plan()->clear();

    std::vector<double> goal_vec = goals[(i + 1) % goals.size()];
    PRX_DEBUG_ITERABLE(goal_vec);
    omnibot_controller->set_goal(goal_vec);
    // write_to_file = true;

    sg->propagate(start_state, omnibot_controller, checker, traj_real);
    accum_traj += traj_real;
    for (auto state : traj_real)
    {
      ofs_traj_real << simulation_step << " " << state << " " << i << "\n";
    }
    // write_to_file = false;

    traj_real.to_file(out_path + "friction_maps/omnirobot_mecanum_fixed_goals_traj_" + std::to_string(i) + ".txt");
    omnibot_controller->get_plan()->to_file(out_path + "friction_maps/omnirobot_mecanum_fixed_goals_plan_" +
                                            std::to_string(i) + ".txt");

    ss->copy(start_state, goal_vec);

    ofs_traj_real << "\n";
  }

  std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();

  three_js_group_t* vis_group = new three_js_group_t({ plant }, { obstacle_list });

  vis_group->add_detailed_vis_infos(info_geometry_t::LINE, traj_real, body_name, ss);

  vis_group->add_animation(traj_real, ss, start_state);

  vis_group->output_html("fixed_goals.html");

  delete vis_group;
}