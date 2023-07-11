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

#include "prx/factor_graphs/factors/euclidian_distance_factor.hpp"
#include "prx/factor_graphs/factors/ackermann_factors.hpp"
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
using namespace prx::fg;
using namespace prx::utilities;

std::unordered_map<std::string, gtsam::noiseModel::Base::shared_ptr> noise_models;

const double WHEEL_DISTANCE{ 0.115 * 2.0 };
const double MASS{ 0.498952 + 3.542137 + 0.1 };

const prx_symbol_t k_params{ symbol_factory_t::create_hashed_symbol("Theta", 0) };

const fg::ackermann::U u_init{ fg::ackermann::U::Zero() };
const fg::ackermann::Q q_init{ fg::ackermann::Q::Zero() };
const fg::ackermann::Qdot qdot_init{ fg::ackermann::Qdot::Zero() };
const fg::ackermann::Qdotdot qdotdot_init{ fg::ackermann::Qdotdot::Zero() };
const fg::ackermann::Force force_init{ fg::ackermann::Force::Zero() };

void create_ackermann_at_idx_fg(gtsam::NonlinearFactorGraph& graph, gtsam::Values& values, const std::size_t idx)
{
  const prx_symbol_t k_u{ symbol_factory_t::create_hashed_symbol("U", idx) };
  const prx_symbol_t k_q0{ symbol_factory_t::create_hashed_symbol("X", idx) };
  const prx_symbol_t k_q1{ symbol_factory_t::create_hashed_symbol("X", idx + 1) };
  const prx_symbol_t k_qdot{ symbol_factory_t::create_hashed_symbol("Xdot", idx) };
  const prx_symbol_t k_qdotdot{ symbol_factory_t::create_hashed_symbol("Xdotdot", idx) };

  const prx_symbol_t k_force{ symbol_factory_t::create_hashed_symbol("F", idx) };

  graph.add(ackermann_q_qdot_u_t(k_q0, k_q1, k_qdot, k_u, noise_models["f1"]));
  graph.add(ackermann_q_qdot_qdotdot_t(k_qdot, k_qdotdot, k_q0, noise_models["f2"], WHEEL_DISTANCE));
  graph.add(ackermann_qdotdot_force_q_t(k_qdotdot, k_force, k_q0, k_params, noise_models["f3"], WHEEL_DISTANCE, MASS));

  // values.insert(k_u, u_init);
  values.insert(k_q0, q_init);
  values.insert(k_qdot, qdot_init);
  values.insert(k_qdotdot, qdotdot_init);
  values.insert(k_force, force_init);
}

int main(int argc, char* argv[])
{
  auto params = param_loader("executables/factor_graphs/ackermann_fg.yaml", argc, argv);
  simulation_step = params["simulation_step"].as<double>();

  // std::string data_file{ params["tensegrity_data"].as<>() };
  // std::string graph_file{ prx::out_path + "/tensegrity/fix_fg.dot" };
  // std::cout << data_file << std::endl;
  // prx::utilities::csv_reader_t reader(data_file);

  noise_models["u_prior"] = gtsam::noiseModel::Isotropic::Sigma(fg::ackermann::DimU, 1e-5);
  noise_models["q_prior"] = gtsam::noiseModel::Isotropic::Sigma(fg::ackermann::DimQ, 1e-3);
  noise_models["f1"] = gtsam::noiseModel::Isotropic::Sigma(fg::ackermann::DimQ, 1e-0);
  noise_models["f2"] = gtsam::noiseModel::Isotropic::Sigma(fg::ackermann::DimQdot, 1e-0);
  noise_models["f3"] = gtsam::noiseModel::Isotropic::Sigma(fg::ackermann::DimQdotdot, 1e-0);

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  const fg::ackermann::Params params_init{ 0.1, 0.50 };
  values.insert(k_params, params_init);
  // while (reader.has_next_line())

  const std::size_t T{ 5'000 };
  graph.addPrior(symbol_factory_t::create_hashed_symbol("X", 0), q_init, noise_models["q_prior"]);
  fg::ackermann::U u_rand{ fg::ackermann::U::Random() };
  u_rand[0] = 1;
  u_rand[1] = 0.52;
  for (std::size_t i = 0; i < T; ++i)
  {
    create_ackermann_at_idx_fg(graph, values, i);

    if (i % 100 == 0)
    {
      u_rand = fg::ackermann::U::Random();
      u_rand[1] *= 0.5;
    }
    const prx_symbol_t k_u{ symbol_factory_t::create_hashed_symbol("U", i) };
    graph.addPrior(k_u, u_rand, noise_models["u_prior"]);
    values.insert_or_assign(k_u, u_rand);
  }
  values.insert(symbol_factory_t::create_hashed_symbol("X", T), q_init);

  gtsam::LevenbergMarquardtParams lm_params{ fg::utilities::default_levenberg_marquardt_parameters() };
  lm_params.verbosityLMTranslator(gtsam::LevenbergMarquardtParams::SILENT);
  lm_params.setMaxIterations(50);
  // lm_params.setMaxIterations(10000);
  lm_params.setRelativeErrorTol(1e-10);
  lm_params.setAbsoluteErrorTol(1e-10);
  lm_params.setlambdaUpperBound(1e64);
  lm_params.print("lm_params");
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);

  logger_t logger(out_path + "ackermann_fg.log");
  gtsam::Values results = fg::utilities::optimize_and_log(optimizer, lm_params, logger, 0);

  logger_t results_logger(out_path + "ackermann_fg_results.txt");
  for (std::size_t i = 0; i < T; ++i)
  {
    const auto r_U{ results.at<fg::ackermann::U>(symbol_factory_t::create_hashed_symbol("U", i)) };
    const auto r_Q{ results.at<fg::ackermann::Q>(symbol_factory_t::create_hashed_symbol("X", i)) };
    const auto r_Qdot{ results.at<fg::ackermann::Qdot>(symbol_factory_t::create_hashed_symbol("Xdot", i)) };
    const auto r_Qdotdot{ results.at<fg::ackermann::Qdotdot>(symbol_factory_t::create_hashed_symbol("Xdotdot", i)) };
    const auto r_Force{ results.at<fg::ackermann::Force>(symbol_factory_t::create_hashed_symbol("F", i)) };

    results_logger("U", i, r_U.transpose());
    results_logger("Q", i, r_Q.transpose());
    results_logger("Qdot", i, r_Qdot.transpose());
    results_logger("Qdotdot", i, r_Qdotdot.transpose());
    results_logger("Force", i, r_Force.transpose());
  }
  const auto r_QT{ results.at<fg::ackermann::Q>(symbol_factory_t::create_hashed_symbol("X", T)) };
  const auto r_Params{ results.at<fg::ackermann::Params>(symbol_factory_t::create_hashed_symbol("Theta", 0)) };
  results_logger("Params", r_Params.transpose());
  results_logger("Q", T, r_QT.transpose());

  return 0;
}