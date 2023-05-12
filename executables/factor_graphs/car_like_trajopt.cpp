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
#include "prx/simulation/playback/utils.hpp"
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
#include "prx/factor_graphs/utilities/constants.hpp"
#include "prx/factor_graphs/utilities/utilities_functions.hpp"
#include "prx/factor_graphs/utilities/formatter.hpp"

#include "prx/factor_graphs/graphs/ilqr.hpp"

#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/linear/JacobianFactor.h>

using prx::symbol_factory_t;

const std::size_t X_DIM{ 5 };
const std::size_t U_DIM{ 2 };

using X = Eigen::Vector<double, X_DIM>;
using U = Eigen::Vector<double, U_DIM>;

using Q = Eigen::Matrix<double, X_DIM, X_DIM>;
using R = Eigen::Matrix<double, U_DIM, U_DIM>;

int main(int argc, char* argv[])
{
  auto params = prx::param_loader("executables/factor_graphs/car_like_trajopt.yaml", argc, argv);

  prx::simulation_step = params["simulation_step"].as<double>();
  prx::init_random(params["random_seed"].as<int>());

  std::string plant_name{ params["/plant/name"].as<>() };
  std::string plant_path{ params["/plant/path"].as<>() };

  prx::system_ptr_t system = prx::system_factory_t::create_system(plant_name, plant_path);
  auto plant = std::dynamic_pointer_cast<prx::plant_t>(system);
  prx::world_model_t world_model({ plant }, {});
  world_model.create_context("context", { plant_name }, {});

  prx::world_model_context_t context = world_model.get_context("context");
  std::shared_ptr<prx::system_group_t> sg{ prx::world_model_t::get_system_group(context) };

  prx::space_t* ss = sg->get_state_space();
  prx::space_t* cs = sg->get_control_space();
  prx::space_t* ps = sg->get_parameter_space();
  const std::size_t ss_dim{ ss->get_dimension() };
  const std::size_t cs_dim{ cs->get_dimension() };
  const std::size_t ps_dim{ ps->get_dimension() };

  prx_assert(ss != nullptr, "Space is null!!!");
  prx_assert(cs != nullptr, "Space is null!!!");
  prx_assert(ps != nullptr, "Space is null!!!");

  auto lower_bounds = params["/plant/state_space_lower_bound"].as<std::vector<double>>();
  auto upper_bounds = params["/plant/state_space_upper_bound"].as<std::vector<double>>();
  ss->set_bounds(lower_bounds, upper_bounds);

  lower_bounds = params["/plant/control_space_lower_bound"].as<std::vector<double>>();
  upper_bounds = params["/plant/control_space_upper_bound"].as<std::vector<double>>();
  cs->set_bounds(lower_bounds, upper_bounds);

  prx::space_point_t c_state = cs->make_point();
  prx::space_point_t s_state = ss->make_point();

  prx::plan_t plan(cs);
  prx::trajectory_t traj(ss);
  std::vector<std::string> plan_files(params["input_plans"].as<std::vector<std::string>>());
  std::vector<std::string> traj_files(params["input_trajs"].as<std::vector<std::string>>());
  std::size_t total_trajs{ traj_files.size() };
  prx_assert(plan_files.size() == total_trajs, "Different number of plans and trajs");

  gtsam::Values values;
  gtsam::NonlinearFactorGraph graph;
  gtsam::LevenbergMarquardtParams lm_params{ prx::fg::utilities::default_levenberg_marquardt_parameters() };

  std::unordered_map<std::string, gtsam::noiseModel::Base::shared_ptr> noise_models;
  noise_models["fix_state"] = gtsam::noiseModel::Isotropic::Sigma(X_DIM, 1e-4);
  noise_models["state"] = gtsam::noiseModel::Isotropic::Sigma(X_DIM, 1e0);
  noise_models["control"] = gtsam::noiseModel::Isotropic::Sigma(U_DIM, 1e-3);
  noise_models["propagation"] = gtsam::noiseModel::Isotropic::Sigma(X_DIM, 1e-1);

  std::vector<prx::prx_symbol_t> control_symbols;
  std::vector<prx::prx_symbol_t> state_symbols;

  prx::prx_symbol_t state_symbol;
  prx::prx_symbol_t control_symbol;
  prx::prx_symbol_t next_state_symbol;

  for (int traj_id = 0; traj_id < total_trajs; ++traj_id)
  {
    plan.clear();
    traj.clear();
    plan.from_file(plan_files[traj_id]);
    traj.from_file(traj_files[traj_id]);
    plan.expand();

    prx::simulation::plan_trajectory_stepper_t plan_trajectory(&plan, &traj);
    for (auto step_states_tuple : plan_trajectory)
    {
      const prx::space_point_t xi = std::get<0>(step_states_tuple);
      const prx::plan_step_t step_i = std::get<1>(step_states_tuple);
      const prx::space_point_t xip1 = std::get<2>(step_states_tuple);
      const std::size_t idx = std::get<3>(step_states_tuple);

      const X state{ xi->vector() };
      const U control{ step_i.control->vector() };
      const double duration{ step_i.duration };

      state_symbol = symbol_factory_t::create_hashed_symbol("state_symbol", idx, traj_id);
      control_symbol = symbol_factory_t::create_hashed_symbol("control_symbol", idx, traj_id);
      next_state_symbol = symbol_factory_t::create_hashed_symbol("state_symbol", idx + 1, traj_id);

      values.insert(state_symbol, state);
      values.insert(control_symbol, control);

      if (idx == 0 && traj_id == 0)
      {
        graph.addPrior(state_symbol, state, noise_models["fix_state"]);
      }
      else
      {
        graph.addPrior(state_symbol, state, noise_models["state"]);
      }
      graph.addPrior(control_symbol, control, noise_models["control"]);

      graph.add(prx::propagation_factor_XU_t<X_DIM, U_DIM>(state_symbol, next_state_symbol, control_symbol, duration,
                                                           noise_models["propagation"], sg));

      // graph.add(prx::fg::quadratic_cost_factor_t<X_DIM>(state_symbol, state, Q::Ones(), 1e0));
      // graph.add(prx::fg::quadratic_cost_factor_t<U_DIM>(control_symbol, control, R::Ones(), 1e0));
      //
      state_symbols.push_back(state_symbol);
      control_symbols.push_back(control_symbol);
    }

    const X state_T{ traj.back()->vector() };
    values.insert(next_state_symbol, state_T);

    if (traj_id < total_trajs - 1)
    {
      graph.addPrior(next_state_symbol, state_T, noise_models["state"]);
      // prx::prx_symbol_t state_symbol{ symbol_factory_t::create_hashed_symbol("state_symbol", traj.size() - 1,
      // traj_id)
      control_symbol = symbol_factory_t::create_hashed_symbol("control_symbol", plan.size(), traj_id);
      next_state_symbol = symbol_factory_t::create_hashed_symbol("state_symbol", 0, traj_id + 1);

      const U control{ U::Zero(cs_dim) };
      const double duration{ prx::simulation_step };
      graph.add(prx::propagation_factor_XU_t<X_DIM, U_DIM>(state_symbol, next_state_symbol, control_symbol, duration,
                                                           noise_models["propagation"], sg));

      // values.insert(state_symbol, state);
      values.insert(control_symbol, control);
    }
  }
  const X state_T{ traj.back()->vector() };
  graph.addPrior(next_state_symbol, state_T, noise_models["fix_state"]);

  gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
  gtsam::Values results = optimizer.optimize();
  symbol_factory_t::symbols_to_file(prx::out_path + "car_like_symbols.txt");

  traj.clear();
  plan.clear();
  for (auto ctrl_sym : control_symbols)
  {
    auto u = results.at<U>(ctrl_sym);
    plan.copy_onto_back(u, prx::simulation_step);
  }
  X start = results.at<X>(state_symbols[0]);
  sg->propagate(start, plan, traj);

  traj.to_file(prx::out_path + "resulting_traj.txt");
}