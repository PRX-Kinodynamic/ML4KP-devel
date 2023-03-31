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

#include "prx/mujoco/mj_simulator.hpp"

#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/linear/JacobianFactor.h>

using namespace prx;
using namespace prx::utilities;
using namespace prx::fg;

using MushrState = Eigen::Vector<double, 27>;

const std::size_t X_DIM{ 27 };
const std::size_t U_DIM{ 2 };
int main(int argc, char* argv[])
{
  auto params = param_loader("executables/factor_graphs/ackermann_traj_from_path.yaml", argc, argv);
  init_random(params["random_seed"].as<int>());

  std::shared_ptr<mujoco_simulator_t> sim =
      std::make_shared<mujoco_simulator_t>("mushr.xml", params["visualize"].as<bool>());
  sim->init_simulator();

  auto context = sim->get_context("mujoco");
  auto ss = context.first->get_state_space();
  auto cs = context.first->get_control_space();
  std::shared_ptr<system_group_t> sg = context.first;

  ss->print_topology();
  cs->print_topology();
  prx::trajectory_t traj(ss);
  double x, y, t, theta;
  double x_prev{ 0 };
  double y_prev{ 0 };
  bool first = true;
  const double tau{ 0.01 };

  csv_reader_t reader(params["curve_file"].as<>());
  double roll = 0, pitch = 0, yaw = 0;
  Eigen::Quaterniond quat;
  MushrState mushr_state = MushrState::Zero();
  while (reader.has_next_line())
  {
    auto line = reader.next_line();
    t = std::stod(line[0]);
    x = std::stod(line[1]);
    y = std::stod(line[2]);

    yaw = std::atan2(y - y_prev, x - x_prev);
    if (first)
    {
      yaw = PRX_PI;
      first = false;
    }
    quat = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    // std::cout << "Quaternion" << std::endl << quat.coeffs() << std::endl;
    mushr_state[0] = x;
    mushr_state[1] = y;
    mushr_state[2] = 0;
    mushr_state[3] = quat.w();
    mushr_state[4] = quat.x();
    mushr_state[5] = quat.y();
    mushr_state[6] = quat.z();
    x_prev = x;
    y_prev = y;
    ss->copy_from(mushr_state);
    sim->step_simulation(propagate_step::FIRST_STEP);
    traj.copy_onto_back(ss);
    std::cout << "mushr_state: " << mushr_state.transpose() << "\n";
    std::cout << "sim: " << ss->print_memory(3) << "\n";
  }

  traj.to_file(prx::out_path + "traj_mushr_dbg.txt");
  std::cout << traj << std::endl;
  auto dm = gtsam::noiseModel::Isotropic::Sigma(X_DIM, 1e0);
  auto tau_nm = gtsam::noiseModel::Isotropic::Sigma(1, 1e-3);
  // MushrState noise = MushrState::Ones();
  // noise[0] = 1e-1;
  // noise[1] = 1e-1;
  // auto dm = gtsam::noiseModel::Diagonal::Sigmas(noise);

  gtsam::Values values;
  gtsam::NonlinearFactorGraph graph;

  const std::size_t total_states{ traj.size() };
  space_point_t rand_ctrl = cs->make_point();
  cs->copy(rand_ctrl, { -1.0, -1.0 });
  using Tau = Eigen::Vector<double, 1>;
  for (std::size_t i = 0; i < total_states - 1; ++i)
  {
    prx::prx_symbol_t state_symbol = symbol_factory_t::create_symbol("state_symbol", i);
    prx::prx_symbol_t next_state_symbol = symbol_factory_t::create_symbol("state_symbol", i + 1);
    prx::prx_symbol_t control_symbol = symbol_factory_t::create_symbol("control_symbol", i);
    prx::prx_symbol_t time_symbol = symbol_factory_t::create_symbol("time_symbol", i);

    // cs->sample(rand_ctrl);
    values.insert(state_symbol, traj[i]->vector<>());
    values.insert(control_symbol, rand_ctrl->vector<>());
    values.insert(time_symbol, Tau(.1));

    const fg::positive_vector_factor_t<1> positive_duration(time_symbol, tau_nm);
    graph.add(positive_duration);
    graph.add(state_prior_factor_t<X_DIM>(state_symbol, traj[i]->vector<>(), ss, dm));
    // graph.addPrior(state_symbol, traj[i]->vector<>(), dm);
    graph.add(  // no-lint
                // propagation_factor_XU_t<X_DIM, U_DIM>(state_symbol, next_state_symbol, control_symbol, tau, dm, sg));
        propagation_factor_XUTau_t<X_DIM, U_DIM>(state_symbol, next_state_symbol, control_symbol, time_symbol, dm, sg));
  }
  prx::prx_symbol_t state_symbol = symbol_factory_t::create_symbol("state_symbol", total_states - 1);
  graph.addPrior(state_symbol, traj[total_states - 1]->vector<>(), dm);
  values.insert(state_symbol, traj[total_states - 1]->vector<>());

  gtsam::LevenbergMarquardtParams lm_params{ prx::fg::utilities::default_levenberg_marquardt_parameters() };
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
  factor_graph_logger_t friction_map_logger(prx::out_path + "mj_mushr_traj_from_path.log", ' ', "-");
  gtsam::Values results = fg::utilities::optimize_and_log(optimizer, lm_params, friction_map_logger, 0);
  // values.print("Results", prx::key_formatter);

  plan_t plan_res(cs);
  trajectory_t traj_res(ss);
  fg::utilities::values_to_plan_and_traj(results, &traj_res, &plan_res, total_states - 1);

  plan_res.to_file(prx::out_path + "mj_mushr_fg_plan.txt");
  traj_res.to_file(prx::out_path + "mj_mushr_fg_traj.txt");
  std::cout << "Trajectory: \n" << traj_res << std::endl;
  std::cout << "Plan: \n" << plan_res << std::endl;

  sim->toggle_visualizer();
  // for (auto s : traj_res)
  // {
  //   ss->copy_from(s);
  //   sim->step_simulation(propagate_step::FIRST_STEP);
  // }
  auto end = ss->make_point();
  auto start = traj_res.front();
  sg->propagate(start, plan_res, end);
  usleep(int(1e6));
  std::cout << ss->print_point(traj_res.back(), 3) << std::endl;
  std::cout << ss->print_point(end, 3) << std::endl;
}