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
#include "prx/factor_graphs/planning/initialization_trajs_fg.hpp"
#include "prx/factor_graphs/factors/parameter_fusion_factor.hpp"
#include "prx/factor_graphs/factors/positive_vector_factor.hpp"
#include "prx/factor_graphs/factors/state_prior.hpp"
#include "prx/factor_graphs/graphs/ilqr.hpp"
#include "prx/factor_graphs/utilities/constants.hpp"
#include "prx/factor_graphs/utilities/formatter.hpp"

#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
using namespace prx;
using namespace prx::utilities;
using namespace prx::fg;

const std::size_t X_DIM{ 3 };
const std::size_t U_DIM{ 2 };
using Tau = Eigen::Vector<double, 1>;
using Control = Eigen::Vector<double, 2>;
using AckermannState = Eigen::Vector<double, 3>;
// auto tau_nm = gtsam::noiseModel::Constrained::All(1);
auto tau_nm = gtsam::noiseModel::Isotropic::Precision(1, 1e-5);
auto control_nm = gtsam::noiseModel::Isotropic::Sigma(2, 1e-2);
auto state_nm = gtsam::noiseModel::Isotropic::Precision(3, 1e-3);
auto constrained_xy = gtsam::noiseModel::Constrained::MixedSigmas(Eigen::Vector3d(0, 0, 10));
auto constrained3d = gtsam::noiseModel::Constrained::All(3);

struct control_recovery
{
  std::shared_ptr<system_group_t> _sg;
  const space_t* _ss;
  const space_t* _cs;
  trajectory_t trajectory;
  plan_t plan;
  std::size_t idx;
  factor_graph_logger_t _friction_map_logger;
  gtsam::LevenbergMarquardtParams _lm_params;

  control_recovery(std::shared_ptr<system_group_t> sg)
    : _sg(sg)
    , _ss(_sg->get_state_space())
    , _cs(_sg->get_control_space())
    , trajectory(_ss)
    , plan(_cs)
    , idx(0)
    , _friction_map_logger(prx::out_path + "ackermannFO_prop_test.log", ' ', "-")
    , _lm_params(prx::fg::utilities::default_levenberg_marquardt_parameters())
  {
    _lm_params.setlambdaFactor(.10);
    _lm_params.setVerbosityLM("TRYLAMBDA");
  }

  template <typename State>
  void controls_estimation_init(State s0, State s1)
  {
    gtsam::Values values;
    gtsam::NonlinearFactorGraph graph;

    prx::prx_symbol_t state_symbol{ symbol_factory_t::create_symbol("state_symbol", idx) };
    prx::prx_symbol_t next_state_symbol{ symbol_factory_t::create_symbol("state_symbol", idx + 1) };
    prx::prx_symbol_t control_symbol{ symbol_factory_t::create_symbol("control_symbol", idx) };
    prx::prx_symbol_t time_symbol{ symbol_factory_t::create_symbol("time_symbol", idx) };

    values.insert(state_symbol, s0);
    values.insert(next_state_symbol, s1);
    values.insert(control_symbol, Control(0.0, 0.0));
    values.insert(time_symbol, Tau(0.1));

    graph.add(fg::positive_vector_factor_t<1>(time_symbol, tau_nm));
    graph.add(state_prior_factor_t<X_DIM>(state_symbol, s0, _ss, constrained3d));
    graph.add(state_prior_factor_t<X_DIM>(next_state_symbol, s1, _ss, constrained_xy));
    graph.add(propagation_factor_XUTau_t<X_DIM, U_DIM>(state_symbol, next_state_symbol, control_symbol, time_symbol,
                                                       constrained_xy, _sg));

    // values = gtsam::DoglegOptimizer(graph, values).optimize();
    // values = gtsam::GaussNewtonOptimizer(graph, values).optimize();
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, _lm_params);
    values = fg::utilities::optimize_and_log(optimizer, _lm_params, _friction_map_logger, idx);

    idx++;
    trajectory.copy_onto_back(values.at<State>(state_symbol));
    trajectory.copy_onto_back(values.at<State>(next_state_symbol));
    plan.copy_onto_back(values.at<Control>(control_symbol), values.at<Tau>(time_symbol)[0]);
  }

  template <typename State>
  void controls_estimation_add_state(State si)
  {
    gtsam::Values values;
    gtsam::NonlinearFactorGraph graph;

    prx::prx_symbol_t state_symbol{ symbol_factory_t::create_symbol("state_symbol", idx) };
    prx::prx_symbol_t next_state_symbol{ symbol_factory_t::create_symbol("state_symbol", idx + 1) };
    prx::prx_symbol_t control_symbol{ symbol_factory_t::create_symbol("control_symbol", idx) };
    prx::prx_symbol_t time_symbol{ symbol_factory_t::create_symbol("time_symbol", idx) };

    const State s0{ trajectory.back()->vector<State>() };
    values.insert(state_symbol, s0);
    values.insert(next_state_symbol, si);
    values.insert(control_symbol, Control(0.0, 0.0));
    values.insert(time_symbol, Tau(0.1));

    graph.add(fg::positive_vector_factor_t<1>(time_symbol, tau_nm));
    graph.add(state_prior_factor_t<X_DIM>(state_symbol, s0, _ss, constrained3d));
    graph.add(state_prior_factor_t<X_DIM>(next_state_symbol, si, _ss, constrained_xy));
    graph.add(propagation_factor_XUTau_t<X_DIM, U_DIM>(state_symbol, next_state_symbol, control_symbol, time_symbol,
                                                       constrained3d, _sg));

    gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, _lm_params);
    values = fg::utilities::optimize_and_log(optimizer, _lm_params, _friction_map_logger, idx);

    trajectory.copy_onto_back(values.at<State>(next_state_symbol));
    plan.copy_onto_back(values.at<Control>(control_symbol), values.at<Tau>(time_symbol)[0]);
    idx++;
  }
};

int main(int argc, char* argv[])
{
  auto params = param_loader("executables/factor_graphs/ackermann_traj_from_path.yaml", argc, argv);
  prx::simulation_step = 0.01;
  std::string plant_name = params["/plant/name"].as<>();
  std::string plant_path = params["/plant/path"].as<>();
  auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
  prx_assert(plant != nullptr, "Plant is nullptr!");

  world_model_t world_model({ plant }, {});
  world_model.create_context("context", { plant_name }, {});
  auto context = world_model.get_context("context");
  auto sg = world_model_t::get_system_group(context);

  const space_t* ss = sg->get_state_space();
  const space_t* cs = sg->get_control_space();

  trajectory_t traj_nominal(ss);

  csv_reader_t reader(params["curve_file"].as<>());
  double x, y, t, theta;
  double x_prev{ 0 };
  double y_prev{ 0 };
  bool first = true;

  const int total_states{ 2 };
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

    traj_nominal.copy_onto_back(Eigen::Vector3d(x, y, theta));
    x_prev = x;
    y_prev = y;
    if (traj_nominal.size() >= total_states)
      break;
  }
  // Eigen::Vector3d s0(0.00000, 0.00000, 3.14159);
  // Eigen::Vector3d s1(-0.08983, 0.00500, 3.08604);
  // Eigen::Vector3d s2(-0.17867, 0.01993, 2.97500);
  // traj_nominal.copy_onto_back(s0);
  // traj_nominal.copy_onto_back(s1);
  // traj_nominal.copy_onto_back(s2);
  traj_nominal.to_file(prx::out_path + "traj_nominal_dbg.txt");

  plan_t plan(cs);
  trajectory_t traj(ss);
  std::size_t idx{ 0 };

  gtsam::Values values;
  gtsam::NonlinearFactorGraph graph;

  control_recovery cr(sg);
  cr.controls_estimation_init(traj_nominal[0]->vector<AckermannState>(), traj_nominal[1]->vector<AckermannState>());
  for (int i = 2; i < traj_nominal.size(); ++i)
  {
    cr.controls_estimation_add_state(traj_nominal[i]->vector<AckermannState>());
  }
  // cr.controls_estimation_add_state(traj_nominal[2]->vector<AckermannState>());
  // s1 = controls_estimation_init(graph, values, s0, s1, idx, ss);
  // prx::fg::utilities::extract_plan_from_values<Eigen::Vector2d, Tau>(values, plan, 1);
  // PRX_DEBUG_VAR_1(plan);
  // graph.erase(graph.begin(), graph.end());
  // values.clear();
  // s2 = controls_estimation_init(graph, values, s1, s2, idx, ss);

  // PRX_DEBUG_VAR_2(s1.transpose(), s2.transpose());

  // prx::fg::utilities::extract_plan_from_values<Eigen::Vector2d, Tau>(values, plan, 1);

  auto start = cr.trajectory.front();
  sg->propagate(start, cr.plan, traj);

  PRX_DEBUG_VAR_1(cr.plan);
  PRX_DEBUG_VAR_1(traj);

  traj.to_file(prx::out_path + "traj_dbg.txt");
  cr.trajectory.to_file(prx::out_path + "traj2_dbg.txt");
}