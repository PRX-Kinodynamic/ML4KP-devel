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
#include "prx/factor_graphs/factors/free_body_dyn_factors.hpp"
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

const fbd::X x_init{ fbd::X::Zero() };
const fbd::Xdot xdot_init{ fbd::Xdot::Zero() };
const fbd::Xdotdot xdotdot_init{ fbd::Xdotdot::Zero() };
const fbd::P p_init{ fbd::P::Zero() };
const fbd::Force force_init{ fbd::Force::Zero() };

const double mass{ 1.0 };

void create_linear_fg(gtsam::NonlinearFactorGraph& graph, gtsam::Values& values, const std::size_t idx, const double dt)
{
  const prx_symbol_t k_x0{ symbol_factory_t::create_hashed_symbol("X", idx) };
  const prx_symbol_t k_x1{ symbol_factory_t::create_hashed_symbol("X", idx + 1) };
  const prx_symbol_t k_xdot0{ symbol_factory_t::create_hashed_symbol("Xdot", idx) };
  const prx_symbol_t k_xdot1{ symbol_factory_t::create_hashed_symbol("Xdot", idx + 1) };
  const prx_symbol_t k_xdotdot{ symbol_factory_t::create_hashed_symbol("Xdotdot", idx) };

  const prx_symbol_t k_p0{ symbol_factory_t::create_hashed_symbol("P", idx) };
  const prx_symbol_t k_p1{ symbol_factory_t::create_hashed_symbol("P", idx + 1) };
  const prx_symbol_t k_force{ symbol_factory_t::create_hashed_symbol("F", idx) };

  graph.add(propagation_euler_factor_t<fbd::X, fbd::Xdot>(k_x0, k_x1, k_xdot0, noise_models["propagation"], dt));
  graph.add(
      propagation_euler_factor_t<fbd::Xdot, fbd::Xdotdot>(k_xdot0, k_xdot1, k_xdotdot, noise_models["propagation"]));
  graph.add(propagation_euler_factor_t<fbd::P, fbd::Force>(k_p0, k_p1, k_force, noise_models["propagation"]));
  graph.add(velocity_linear_momentum_factor_t(k_xdot0, k_p0, noise_models["linear"], mass));
  graph.add(force_acceleration_factor_t(k_xdotdot, k_force, noise_models["linear"], mass));

  values.insert(k_x0, x_init);
  values.insert(k_xdot0, xdot_init);
  values.insert(k_xdotdot, xdotdot_init);
  values.insert(k_p0, p_init);
  values.insert(k_force, force_init);
}

int main(int argc, char* argv[])
{
  auto params = param_loader("executables/tensegrity/imu_fg.yaml", argc, argv);
  simulation_step = params["simulation_step"].as<double>();

  std::string imu_file{ params["imu_file"].as<>() };
  std::string graph_file{ prx::out_path + "/tensegrity/fix_fg.dot" };
  std::cout << imu_file << std::endl;
  prx::utilities::csv_reader_t reader(imu_file);

  noise_models["x_prior"] = gtsam::noiseModel::Isotropic::Sigma(fbd::DimX, 1e-3);
  noise_models["propagation"] = gtsam::noiseModel::Isotropic::Sigma(3, 1e-0);
  noise_models["linear"] = gtsam::noiseModel::Isotropic::Sigma(3, 1e-0);

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  std::size_t idx{ 0 };
  double t_prev{ 0.0 };
  const fbd::X x0{ fbd::X::Zero() };
  graph.addPrior(symbol_factory_t::create_hashed_symbol("X", idx), x0, noise_models["x_prior"]);

  while (reader.has_next_line())
  {
    auto line = reader.next_line<double>();
    if (line.size() == 0)
      continue;
    // PRX_DEBUG_ITERABLE(line);

    const double dt{ line[0] - t_prev };
    create_linear_fg(graph, values, idx, dt);
    t_prev = line[0];

    const prx_symbol_t k_force{ symbol_factory_t::create_hashed_symbol("F", idx) };
    const fbd::Force force_idx{ line[1], line[2], line[3] };
    values.insert_or_assign(k_force, force_idx);

    idx++;
  }
  symbol_factory_t::symbols_to_file();

  values.insert(symbol_factory_t::create_hashed_symbol("X", idx), x_init);
  values.insert(symbol_factory_t::create_hashed_symbol("Xdot", idx), xdot_init);
  values.insert(symbol_factory_t::create_hashed_symbol("P", idx), p_init);

  gtsam::LevenbergMarquardtParams lm_params{ fg::utilities::default_levenberg_marquardt_parameters() };
  lm_params.verbosityLMTranslator(gtsam::LevenbergMarquardtParams::SUMMARY);
  lm_params.setMaxIterations(50);
  // lm_params.setMaxIterations(10000);
  lm_params.setRelativeErrorTol(1e-10);
  lm_params.setAbsoluteErrorTol(1e-10);
  lm_params.setlambdaUpperBound(1e64);
  lm_params.print("lm_params");
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);

  logger_t logger(out_path + "ackermann_fg.log");
  gtsam::Values results = fg::utilities::optimize_and_log(optimizer, lm_params, logger, 0);

  logger_t results_logger(out_path + "imu_results.txt");
  for (std::size_t i = 0; i < idx; ++i)
  {
    const auto Xi{ results.at<fbd::X>(symbol_factory_t::create_hashed_symbol("X", i)) };

    results_logger("X", i, Xi.transpose());
  }
  return 0;
}