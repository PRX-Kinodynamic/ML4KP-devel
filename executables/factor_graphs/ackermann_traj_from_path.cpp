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
#include "prx/utilities/general/csv_reader.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/utilities/general/range.hpp"
#include "prx/utilities/geometry/regular_grid.hpp"

#include "prx/visualization/three_js_group.hpp"

#include "prx/factor_graphs/defs.hpp"
#include "prx/factor_graphs/utilities/fg_logger.hpp"
#include "prx/factor_graphs/planning/initialization_trajs_fg.hpp"
#include "prx/factor_graphs/factors/parameter_fusion_factor.hpp"

#include "prx/factor_graphs/factors/positive_vector_factor.hpp"
#include "prx/factor_graphs/factors/state_prior.hpp"
#include "prx/factor_graphs/graphs/ilqr.hpp"
#include "prx/factor_graphs/utilities/constants.hpp"
#include "prx/factor_graphs/utilities/formatter.hpp"
#include "prx/factor_graphs/utilities/utilities_functions.hpp"

#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/linear/JacobianFactor.h>

using namespace prx;
using namespace prx::utilities;
using namespace prx::fg;

const std::size_t X_DIM{ 3 };
const std::size_t U_DIM{ 2 };
int main(int argc, char* argv[])
{
  auto params = param_loader("executables/factor_graphs/ackermann_traj_from_path.yaml", argc, argv);

  csv_reader_t reader(params["curve_file"].as<>());

  std::string plant_name = params["/plant/name"].as<>();
  std::string plant_path = params["/plant/path"].as<>();
  auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
  prx_assert(plant != nullptr, "Plant is nullptr!");

  world_model_t world_model({ plant }, {});
  world_model.create_context("context", { plant_name }, {});
  auto context = world_model.get_context("context");
  std::shared_ptr<system_group_t> sg = world_model_t::get_system_group(context);

  const space_t* ss = sg->get_state_space();
  const space_t* cs = sg->get_control_space();

  prx::plan_t plan(cs);
  prx::trajectory_t traj(ss);
  double x, y, t, theta;
  double x_prev{ 0 };
  double y_prev{ 0 };
  bool first = true;
  const double tau{ 0.01 };
  space_point_t rand_ctrl = cs->make_point();
  cs->copy(rand_ctrl, { -.80, 0.1 });
  while (reader.has_next_line())
  {
    auto line = reader.next_line();
    t = std::stod(line[0]);
    x = std::stod(line[1]);
    y = std::stod(line[2]);

    theta = std::atan2(y - y_prev, x - x_prev);
    if (first)
    {
      theta = PRX_PI;
      first = false;
    }
    // PRX_DEBUG_VAR_3(x, x_prev, x - x_prev);
    // PRX_DEBUG_VAR_3(y, y_prev, y - y_prev);
    // PRX_DEBUG_VAR_1(theta);
    // PRX_DEBUG_VAR_2((x - x_prev) / (y - y_prev), std::acos((x - x_prev) / (y - y_prev)));
    traj.copy_onto_back(Eigen::Vector3d(x, y, theta));
    plan.copy_onto_back(rand_ctrl, 0.1);
    x_prev = x;
    y_prev = y;
  }
  plan.pop_back();
  traj.to_file(prx::out_path + "traj_dbg.txt");
  std::cout << traj << std::endl;
  // auto dm = gtsam::noiseModel::Isotropic::Sigma(X_DIM, 1e0);
  // auto dm = gtsam::noiseModel::Constrained::MixedSigmas(Eigen::Vector3d(100, 100, 1), Eigen::Vector3d(0, 0, 1));
  // auto dm = gtsam::noiseModel::Constrained::All(3);
  auto tau_nm = gtsam::noiseModel::Constrained::All(1);
  auto dm = gtsam::noiseModel::Diagonal::Sigmas(Eigen::Vector3d(1e-3, 1e-3, 1e0));
  auto start = traj.front();
  plan_t plan_res(cs);
  trajectory_t traj_fg(ss);
  trajectory_t traj_res(ss);
  using Tau = Eigen::Vector<double, 1>;
  for (int i = 0; i < 10; ++i)
  {
    gtsam::Values values;
    gtsam::NonlinearFactorGraph graph;

    const std::size_t total_states{ traj.size() };
    for (std::size_t i = 0; i < total_states - 1; ++i)
    {
      prx::prx_symbol_t state_symbol = symbol_factory_t::create_symbol("state_symbol", i);
      prx::prx_symbol_t next_state_symbol = symbol_factory_t::create_symbol("state_symbol", i + 1);
      prx::prx_symbol_t control_symbol = symbol_factory_t::create_symbol("control_symbol", i);
      prx::prx_symbol_t time_symbol = symbol_factory_t::create_symbol("time_symbol", i);

      // cs->sample(rand_ctrl);
      values.insert(state_symbol, traj[i]->vector<>());
      values.insert(control_symbol, plan[i].control->vector<>());
      values.insert(time_symbol, Tau(plan[i].duration));

      const fg::positive_vector_factor_t<1> positive_duration(time_symbol, tau_nm);
      graph.add(positive_duration);
      graph.add(state_prior_factor_t<X_DIM>(state_symbol, traj[i]->vector<>(), ss, dm));
      // graph.addPrior(state_symbol, traj[i]->vector<>(), dm);
      graph.add(propagation_factor_XUTau_t<X_DIM, U_DIM>(state_symbol, next_state_symbol, control_symbol, time_symbol,
                                                         dm, sg));
      // propagation_factor_XU_t<X_DIM, U_DIM>(state_symbol, next_state_symbol, control_symbol, 0.01, dm, sg));
      graph.add(quadratic_cost_factor_t<X_DIM>(state_symbol, traj[i]->vector<>(), Eigen::Matrix3d::Ones(), 1e0));
      graph.add(quadratic_cost_factor_t<U_DIM>(control_symbol, rand_ctrl->vector<>(), Eigen::Matrix2d::Ones(), 1e0));
    }
    prx::prx_symbol_t state_symbol = symbol_factory_t::create_symbol("state_symbol", total_states - 1);
    graph.addPrior(state_symbol, traj[total_states - 1]->vector<>(), dm);
    values.insert(state_symbol, traj[total_states - 1]->vector<>());

    gtsam::LevenbergMarquardtParams lm_params{ prx::fg::utilities::default_levenberg_marquardt_parameters() };
    lm_params.setMaxIterations(100);
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
    factor_graph_logger_t friction_map_logger(prx::out_path + "ackermann_traj_from_path.log", ' ', "-");
    gtsam::Values results = fg::utilities::optimize_and_log(optimizer, lm_params, friction_map_logger, 0);

    fg::utilities::values_to_plan_and_traj(results, &traj_fg, &plan_res, total_states - 1);
    sg->propagate(start, plan_res, traj_res);
    traj = traj_res;
    plan = plan_res;
    plan.expand();
    std::cout << "Plan: \n" << plan << std::endl;
    PRX_DEBUG_VAR_2(traj.size(), plan.size());
  }
  // values.print("Results", prx::key_formatter);
  plan_res.to_file(prx::out_path + "ackermann_fg_plan.txt");
  traj_res.to_file(prx::out_path + "ackermann_fo_traj_res.txt");
  std::cout << "Trajectory: \n" << traj_res << std::endl;
  std::cout << "Plan: \n" << plan_res << std::endl;

  std::string body_name{ params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>() };
  three_js_group_t* vis_group_sln = new three_js_group_t({ plant }, {});
  vis_group_sln->add_detailed_vis_infos(info_geometry_t::FULL_LINE, traj_fg, body_name, ss);
  vis_group_sln->add_animation(traj, ss);
  vis_group_sln->output_html(params["/plant/name"].as<>() + "_ack_initial_traj.html");
  vis_group_sln->reset();

  vis_group_sln->add_detailed_vis_infos(info_geometry_t::FULL_LINE, traj_res, body_name, ss);
  vis_group_sln->add_animation(traj_res, ss);
  vis_group_sln->output_html(params["/plant/name"].as<>() + "_ack_sln_traj.html");
}