#include <fstream>
#include <iostream>
#include <string>
#include <utility>
#include <boost/algorithm/string/join.hpp>
#include <boost/optional.hpp>

#include "prx/utilities/defs.hpp"

#include "prx/simulation/plants/plants.hpp"
#include "prx/simulation/system_factory.hpp"
#include "prx/simulation/playback/trajectory.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"

#include "prx/planning/world_model.hpp"
#include "prx/simulation/system_group.hpp"
#include "prx/planning/planner_statistics.hpp"
#include "prx/planning/loaders/planner_loader.hpp"
#include "prx/visualization/three_js_group.hpp"

#include "prx/factor_graphs/defs.hpp"
#include "prx/factor_graphs/utilities/fg_logger.hpp"
#include "prx/factor_graphs/planning/trajectory_fg.hpp"
#include "prx/factor_graphs/planning/trajectory_optimizer.hpp"
#include "prx/factor_graphs/utilities/utilities_functions.hpp"
#include "prx/factor_graphs/planning/initialization_trajs_fg.hpp"

using namespace prx;

int main(int argc, char* argv[])
{
  auto params = param_loader("executables/factor_graphs/sysid.yaml", argc, argv);

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

  auto context = world_model.get_context("context");
  auto sg = context.first;
  const auto ss = sg->get_state_space();
  const auto cs = sg->get_control_space();
  const auto ps = sg->get_parameter_space();
  prx_assert(ps != nullptr, "Parameter space is null!!!");

  auto ss_dim = ss->get_dimension();
  auto cs_dim = cs->get_dimension();
  auto ps_dim = ps->get_dimension();
  auto cs_lb = params["/plant/control_space_lower_bound"].as<std::vector<double>>();
  auto cs_up = params["/plant/control_space_upper_bound"].as<std::vector<double>>();
  cs->set_bounds(cs_lb, cs_up);

  std::string traj_in_name = params["traj_file_in"].as<>();
  std::string plan_in_name = params["plan_file_in"].as<>();

  trajectory_t traj_in(ss);
  trajectory_t traj_out(ss);
  trajectory_t traj_real(ss);

  traj_in.from_file(traj_in_name);

  auto start_state = ss->make_point();
  auto goal_state = ss->make_point();

  ss->copy_point(start_state, traj_in.front());
  ss->copy_point(goal_state, traj_in.back());

  std::cout << "[IN] traj front: " << traj_in.front() << std::endl;
  std::cout << "[IN] traj back: " << traj_in.back() << std::endl;

  plan_t plan_in(cs);
  plan_t plan_out(cs);

  plan_in.from_file(plan_in_name);

  trajectory_fg_params_t fg_params;
  fg_params.num_steps = traj_in.size();

  auto tfg = trajectory_fg_t(plant);
  // auto graph = tfg.get_recovering_ctrls_fg(traj_in, sg, fg_params);
  gtsam::NonlinearFactorGraph graph;
  auto dm = gtsam::noiseModel::Isotropic::Sigma(ss_dim, 1e0);
  double t = 0;
  for (unsigned i = 0; i < traj_in.size(); ++i)
  {
    if (i < traj_in.size() - 1)
    {
      graph.add(propagation_factor_1_t(
          dm, traj_in[i]->vector<>(), traj_in[i + 1]->vector<>(), plan_in.at(static_cast<double>(t))->vector<>(),
          (Eigen::VectorXd(1) << simulation_step).finished(), symbol_factory_t::create_symbol("param_symbol", 0), sg));
      graph.add(space_limit_factor_t(symbol_factory_t::create_symbol("param_symbol", 0),
                                     gtsam::noiseModel::Isotropic::Sigma(ps_dim, 1e0), ps));
    }
    t += simulation_step;
  }

  gtsam::LevenbergMarquardtParams lm_params;
  lm_params.setVerbosityLM("SUMMARY");
  lm_params.setlambdaUpperBound(1e32);
  lm_params.setUseFixedLambdaFactor(false);
  lm_params.setDiagonalDamping(false);
  lm_params.setlambdaFactor(1);
  lm_params.setlambdaInitial(1e-7);
  lm_params.setMaxIterations(params["max_iterations"].as<int>());
  lm_params.setRelativeErrorTol(1e-9);
  lm_params.setAbsoluteErrorTol(1e-9);

  std::string file_prefix = out_path + "omnibot_trajs/sysid_" + params["/plant/name"].as<>();
  fg_logger_t lg(file_prefix + "_log.txt", ' ', "-");
  // int outer_iters = params["outer_iters"].as<int>();

  gtsam::Values init_vals;
  // auto init_vals = initialization_trajs_fg_t::traj_to_vals(plant, traj_in);

  double step_size = simulation_step * params["num_steps"].as<double>();

  Eigen::VectorXd param_guess = Eigen::VectorXd::Zero(ps->get_dimension());
  init_vals.insert(symbol_factory_t::create_symbol("param_symbol", 0), param_guess);

  int total_iters = 0;
  // int outer_iters = params["outer_iters"].as<int>();

  // for (int i = 0; i < outer_iters; ++i)
  // {
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_vals, lm_params);
  auto results = fg_utilities::optimize_and_log(optimizer, lm_params, lg, total_iters);
  // 	total_iters += optimizer.iterations();
  // 	init_vals = results;
  // 	fg_utilities::values_to_plan_and_traj(init_vals, &traj_out, &plan_out, fg_params.num_steps);
  // 	// fg_utilities::values_to_plan_and_traj(init_vals, &traj_aux, &plan_aux, fg_params.num_steps);
  // 	if (i < outer_iters - 1)
  // 	{
  // 		sg -> propagate(start_state, plan_out, traj_out);
  // 		fg_utilities::updates_values_from_plan_and_traj(init_vals, plant, traj_out, plan_out, fg_params.num_steps);
  // 	}
  // 	last_error = optimizer.error();
  // }
  std::cout << "Done!!!" << std::endl;

  // fg_utilities::values_to_traj(results, traj_out, fg_params.num_steps );
  // fg_utilities::values_to_plan(results, &plan_out, fg_params.num_steps -1 );
  // auto us = symbol_factory_t::create_symbol("control_symbol", i);
  auto theta_learned = results.at<Eigen::VectorXd>(symbol_factory_t::create_symbol("param_symbol", 0));

  std::cout << "theta_learned:\n" << theta_learned.transpose() << std::endl;

  ps->copy_from_vector(theta_learned);

  sg->propagate(start_state, plan_in, traj_real);

  // std::cout << "[OUT] start_state: " << traj_out.front() << std::endl;
  // std::cout << "[OUT] goal_state: " << traj_out.back() << std::endl;

  std::cout << "[REAL] start_state: " << traj_real.front() << std::endl;
  std::cout << "[REAL] goal_state: " << traj_real.back() << std::endl;

  traj_real.to_file(file_prefix + "_real.txt");
  std::cout << "traj_real: " << file_prefix + "_real.txt" << std::endl;
  three_js_group_t* vis_group = new three_js_group_t({ plant }, {});
  std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();

  vis_group->add_detailed_vis_infos(info_geometry_t::FULL_LINE, traj_in, body_name, ss);
  vis_group->add_animation(traj_in, ss, start_state);
  vis_group->output_html(params["/plant/name"].as<>() + "_traj_opt_in.html");

  // vis_group -> reset();
  // vis_group -> add_detailed_vis_infos(info_geometry_t::FULL_LINE, traj_out, body_name, ss);
  // vis_group -> add_animation(traj_out, ss, start_state);
  // vis_group -> output_html(params["/plant/name"].as<>() + "_traj_opt_out.html");

  vis_group->reset();
  vis_group->add_detailed_vis_infos(info_geometry_t::FULL_LINE, traj_real, body_name, ss);
  vis_group->add_animation(traj_real, ss, start_state);
  vis_group->output_html(params["/plant/name"].as<>() + "_traj_opt_real.html");

  delete vis_group;

  return 0;
}