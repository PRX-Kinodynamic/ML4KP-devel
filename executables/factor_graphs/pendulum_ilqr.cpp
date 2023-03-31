#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>

#include <boost/algorithm/string/join.hpp>
#include <boost/optional.hpp>

#include "prx/planning/loaders/planner_loader.hpp"
#include "prx/planning/planners/rrt.hpp"
#include "prx/planning/planner_functions/planner_functions.hpp"
#include "prx/planning/planner_statistics.hpp"
#include "prx/planning/world_model.hpp"

#include "prx/simulation/controllers/custom_controller.hpp"
#include "prx/simulation/general/condition_check.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/simulation/plants/types/noisy_plant.hpp"
#include "prx/simulation/playback/trajectory.hpp"
#include "prx/simulation/system_factory.hpp"
#include "prx/simulation/system_group.hpp"

#include "prx/utilities/defs.hpp"
#include "prx/utilities/geometry/regular_grid.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/utilities/general/range.hpp"

#include "prx/visualization/three_js_group.hpp"

#include "prx/factor_graphs/defs.hpp"
#include "prx/factor_graphs/utilities/fg_logger.hpp"
#include "prx/factor_graphs/planning/initialization_trajs_fg.hpp"
#include "prx/factor_graphs/factors/parameter_fusion_factor.hpp"

#include "prx/factor_graphs/utilities/utilities_functions.hpp"
#include "prx/factor_graphs/utilities/formatter.hpp"

#include "prx/factor_graphs/graphs/ilqr.hpp"

#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/linear/JacobianFactor.h>

using namespace prx;

gtsam::LevenbergMarquardtParams fg_params()
{
  gtsam::LevenbergMarquardtParams lm_params;
  lm_params.setVerbosityLM("SUMMARY");
  lm_params.setlambdaUpperBound(1e32);
  lm_params.setUseFixedLambdaFactor(false);
  lm_params.setDiagonalDamping(true);
  lm_params.setlambdaFactor(2);
  lm_params.setlambdaInitial(1e-7);
  lm_params.setMaxIterations(10);
  lm_params.setRelativeErrorTol(1e-6);
  lm_params.setAbsoluteErrorTol(1e-6);

  // const std::string log_file{ out_path + "friction_maps/fg_concurrent_log.txt" };
  // std::remove(log_file.c_str());
  // lm_params.setLogFile(log_file);

  return lm_params;
}

int main(int argc, char* argv[])
{
  auto params = param_loader("executables/factor_graphs/ilqr.yaml", argc, argv);

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

  world_model_context_t context = world_model.get_context("context");
  std::shared_ptr<system_group_t> sg{ world_model_t::get_system_group(context) };

  const auto ss = sg->get_state_space();
  const auto cs = sg->get_control_space();
  const auto ps = sg->get_parameter_space();
  const auto ss_dim = ss->get_dimension();
  const auto cs_dim = cs->get_dimension();
  const auto ps_dim = ps->get_dimension();

  space_point_t c_state = cs->make_point();
  space_point_t s_state = ss->make_point();

  prx_assert(ps != nullptr, "Parameter space is null!!!");

  auto cs_lb = params["/plant/control_space_lower_bound"].as<std::vector<double>>();
  auto cs_up = params["/plant/control_space_upper_bound"].as<std::vector<double>>();
  cs->set_bounds(cs_lb, cs_up);

  gtsam::LevenbergMarquardtParams lm_params{ fg_params() };
  gtsam::Values results;

  const double trajectory_duration{ params["trajectory_duration"].as<double>() };

  gtsam::Values init_vals;
  gtsam::NonlinearFactorGraph ilqr_graph;
  Eigen::Vector4d param_vector;
  ps->copy_to(param_vector);

  const Eigen::Matrix<double, 2, 2> Q{ Eigen::Matrix<double, 2, 2>::Ones() };
  const Eigen::Matrix<double, 1, 1> R{ Eigen::Matrix<double, 1, 1>::Ones() };

  auto propagation_noise = gtsam::noiseModel::Constrained::All(ss_dim);
  auto initial_state_noise = gtsam::noiseModel::Constrained::All(ss_dim);
  auto param_prior_noise = gtsam::noiseModel::Constrained::All(ps_dim);
  auto time_prior_noise = gtsam::noiseModel::Constrained::All(1);
  auto control_limit_noise = gtsam::noiseModel::Constrained::All(cs_dim);
  auto x_cost_noise = gtsam::noiseModel::Diagonal::Sigmas(Q.diagonal());
  auto x_final_cost_noise = gtsam::noiseModel::Gaussian::Information(R * 10);
  auto u_cost_noise = gtsam::noiseModel::Isotropic::Sigma(1, 1e0);
  // auto cs_dm = gtsam::noiseModel::Isotropic::Sigma(cs_dim, 1e0);
  // auto t_dm = gtsam::noiseModel::Isotropic::Sigma(1, 1e-3);
  // std::size_t state_num{ 0 };
  const std::size_t total_states{ static_cast<std::size_t>(trajectory_duration / simulation_step) };
  PRX_DEBUG_VAR_1(total_states)

  Eigen::VectorXd initial_state{ Eigen::Vector2d::Zero() };
  ss->copy(initial_state, params["/plant/start_state"].as<std::vector<double>>());
  // const Eigen::VectorXd initial_state{ (Eigen::VectorXd(2) << 0.5, 0).finished() };
  const Eigen::VectorXd final_state{ (Eigen::VectorXd(2) << 0, 0).finished() };
  const Eigen::VectorXd goal_state{ (Eigen::VectorXd(2) << 0, 0).finished() };
  const Eigen::VectorXd goal_control{ (Eigen::VectorXd(1) << 0).finished() };

  // const prx_symbol_t initial_state_symbol{ symbol_factory_t::create_symbol("state_symbol", 0) };
  // ilqr_graph.addPrior(initial_state_symbol, initial_state, initial_state_noise);

  plan_t plan(cs);
  trajectory_t traj(ss);

  cs->copy(c_state, { 0.0 });
  ss->copy(s_state, initial_state);

  plan.resize(total_states - 2);
  for (int i = 0; i < total_states - 1; ++i)
  {
    cs->sample(c_state);
    // ss->sample(s_state);
    //   plan.copy_onto_back(c_state, simulation_step);
    traj.copy_onto_back(s_state);
    plan.copy_onto_back(c_state, simulation_step);
  }
  traj.copy_onto_back(final_state);
  // sg->propagate(initial_state, plan, traj);

  // for (std::size_t i = 0; i < total_states; ++i)
  // {
  //   const prx_symbol_t state_symbol{ symbol_factory_t::create_symbol("state_symbol", i) };
  //   const prx_symbol_t next_state_symbol{ symbol_factory_t::create_symbol("state_symbol", i + 1) };
  //   const prx_symbol_t control_symbol{ symbol_factory_t::create_symbol("control_symbol", i) };
  //   const prx_symbol_t time_symbol{ symbol_factory_t::create_symbol("time_symbol", i) };
  //   const prx_symbol_t param_symbol{ symbol_factory_t::create_symbol("param_symbol", i) };

  //   const Eigen::VectorXd init_state{ traj[i]->vector() };
  //   // const Eigen::VectorXd init_next_state{ Eigen::VectorXd::Zero(2) };
  //   const Eigen::VectorXd init_control{ plan[i].control->vector() };
  //   const Eigen::VectorXd init_time{ (Eigen::VectorXd(1) << simulation_step).finished() };

  //   init_vals.insert_or_assign(state_symbol, init_state);
  //   // init_vals.insert_or_assign(next_state_symbol, init_next_state);
  //   init_vals.insert_or_assign(control_symbol, init_control);
  //   init_vals.insert_or_assign(time_symbol, init_time);
  //   init_vals.insert_or_assign(param_symbol, param_vector);

  //   ilqr_graph.add(fg::quadratic_cost_factor_t<2>(state_symbol, goal_state, Q));
  //   // ilqr_graph.add(gtsam::JacobianFactor(state_symbol, Eigen::Matrix<double, 2, 2>::Ones(), goal_state,
  //   // x_cost_noise));
  //   ilqr_graph.add(fg::quadratic_cost_factor_t<1>(control_symbol, goal_control, R));
  //   ilqr_graph.add(propagation_factor_5_t<2, 1, 4>(state_symbol, next_state_symbol, control_symbol, time_symbol,
  //                                                  param_symbol, propagation_noise, sg));

  //   ilqr_graph.addPrior(time_symbol, init_time, time_prior_noise);
  //   ilqr_graph.addPrior(param_symbol, param_vector, param_prior_noise);

  //   ilqr_graph.add(space_limit_factor_t(control_symbol, control_limit_noise, cs));
  //   // ilqr_graph.add(space_limit_factor_t(state_symbol, space_limit_noise, ss));
  // }
  // const prx_symbol_t final_state_symbol{ symbol_factory_t::create_symbol("state_symbol", total_states) };
  // ilqr_graph.addPrior(final_state_symbol, final_state, initial_state_noise);
  // init_vals.insert_or_assign(final_state_symbol, final_state);
  // ilqr_graph.add(quadratic_cost_factor_t<2>(state_symbol, goal_state, Eigen::Matrix2d::Ones()));

  prx::fg::ilqr_parameters_t<2, 1, 4> ilqr_params;
  ilqr_params.start_state = initial_state;
  ilqr_params.goal_state = goal_state;
  ilqr_params.goal_control = goal_control;
  ilqr_params.param_vector = param_vector;
  ilqr_params.Q = Q;
  ilqr_params.R = R;

  std::tie(ilqr_graph, init_vals) = prx::fg::ilqr_factor_graph<2, 1, 4>(traj, plan, sg, ilqr_params);

  gtsam::LevenbergMarquardtOptimizer optimizer(ilqr_graph, init_vals, lm_params);
  fg_logger_t logger(out_path + "/pendulum_ilqr_log.txt", ' ', "-");
  const std::string graph_txt{ out_path + "/ilqr/pendulum_ilqr_graph.txt" };
  const std::string traj_txt{ out_path + "/ilqr/pendulum_ilqr_traj.txt" };
  const std::string plan_txt{ out_path + "/ilqr/pendulum_ilqr_plan.txt" };

  results = fg::utilities::optimize_and_log(optimizer, lm_params, logger, 0);
  fg::formatter_t graph_formatter;
  ilqr_graph.saveGraph(graph_txt, results, prx::key_formatter, graph_formatter);

  fg::utilities::values_to_plan_and_traj(results, &traj, &plan, plan.size());

  traj.to_file(traj_txt);
  plan.to_file(plan_txt);

  trajectory_t traj_real(ss);
  sg->propagate(initial_state, plan, traj_real);

  ss->copy(s_state, initial_state);
  std::string body_name{ params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>() };
  three_js_group_t* vis_group_sln = new three_js_group_t({ plant }, {});
  vis_group_sln->add_detailed_vis_infos(info_geometry_t::FULL_LINE, traj, body_name, ss);
  vis_group_sln->add_animation(traj, ss, s_state);
  vis_group_sln->output_html(params["/plant/name"].as<>() + "_ilqr_sln_traj.html");

  vis_group_sln->reset();
  vis_group_sln->add_detailed_vis_infos(info_geometry_t::FULL_LINE, traj_real, body_name, ss);
  vis_group_sln->add_animation(traj_real, ss, s_state);
  vis_group_sln->output_html(params["/plant/name"].as<>() + "_ilqr_real_traj.html");

  delete vis_group_sln;
}