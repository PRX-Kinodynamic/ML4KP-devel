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

void create_linear_fg(gtsam::NonlinearFactorGraph& graph, const std::size_t idx, const double mass)
{
  const prx_symbol_t k_x0{ symbol_factory_t::create_hashed_symbol("X", idx) };
  const prx_symbol_t k_x1{ symbol_factory_t::create_hashed_symbol("X", idx + 1) };
  const prx_symbol_t k_xdot0{ symbol_factory_t::create_hashed_symbol("Xdot", idx) };
  const prx_symbol_t k_xdot1{ symbol_factory_t::create_hashed_symbol("Xdot", idx + 1) };
  const prx_symbol_t k_xdotdot{ symbol_factory_t::create_hashed_symbol("X", idx) };

  const prx_symbol_t k_p0{ symbol_factory_t::create_hashed_symbol("P", idx) };
  const prx_symbol_t k_p1{ symbol_factory_t::create_hashed_symbol("P", idx + 1) };
  const prx_symbol_t k_force{ symbol_factory_t::create_hashed_symbol("F", idx) };

  graph.add(propagation_euler_factor_t<fbd::X, fbd::Xdot>(k_x0, k_x1, k_xdot0, noise_models["propagation"]));
  graph.add(
      propagation_euler_factor_t<fbd::Xdot, fbd::Xdotdot>(k_xdot0, k_xdot1, k_xdotdot, noise_models["propagation"]));
  graph.add(propagation_euler_factor_t<fbd::P, fbd::Force>(k_p0, k_p1, k_force, noise_models["propagation"]));
  graph.add(velocity_linear_momentum_factor_t(k_xdot0, k_p0, noise_models["linear"], mass));
  graph.add(force_acceleration_factor_t(k_xdotdot, k_force, noise_models["linear"], mass));
}

int main(int argc, char* argv[])
{
  auto params = param_loader("executables/tensegrity/imu_fg.yaml", argc, argv);
  simulation_step = params["simulation_step"].as<double>();

  std::string data_file{ params["tensegrity_data"].as<>() };
  std::string graph_file{ prx::out_path + "/tensegrity/fix_fg.dot" };
  std::cout << data_file << std::endl;
  prx::utilities::csv_reader_t reader(data_file);

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  while (reader.has_next_line())
  {
  }
}