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
#include "prx/utilities/general/constants.hpp"
#include "prx/utilities/general/csv_reader.hpp"
#include "prx/utilities/general/range.hpp"
#include "prx/utilities/geometry/regular_grid.hpp"
#include "prx/utilities/math/first_order_derivative.hpp"

#include "prx/visualization/three_js_group.hpp"

#include "prx/factor_graphs/defs.hpp"
#include "prx/factor_graphs/factors/friction_fusion_factor.hpp"
#include "prx/factor_graphs/factors/mj_friction_propagation.hpp"
#include "prx/factor_graphs/factors/parameter_fusion_factor.hpp"
#include "prx/factor_graphs/factors/positive_vector_factor.hpp"
#include "prx/factor_graphs/factors/state_prior.hpp"
#include "prx/factor_graphs/utilities/constants.hpp"
#include "prx/factor_graphs/utilities/fg_logger.hpp"
#include "prx/factor_graphs/utilities/utilities_functions.hpp"
#include "prx/factor_graphs/utilities/formatter.hpp"

#include "prx/mujoco/mj_simulator.hpp"
#include "prx/mujoco/plants/mj_friction_plant.hpp"
#include <gtsam/nonlinear/Marginals.h>

#include "friction_maps.hpp"

using namespace prx;

const int XMAX{ 20 };
const int YMAX{ 20 };
const int GRID_DIVISIONS{ 10 };
const int BASIS_DIM{ (GRID_DIVISIONS + 1) * (GRID_DIVISIONS + 1) };

const Eigen::Index TH_DIM{ Eigen::Dynamic };

using friction_vector_t = Eigen::Vector<double, TH_DIM>;
using basis_vector_t = Eigen::Vector<double, BASIS_DIM>;

std::string to_zero_lead(const int value, const unsigned precision)
{
  std::ostringstream oss;
  oss << std::setw(precision) << std::setfill('0') << value;
  return oss.str();
}

int main(int argc, char** argv)
{
  const std::string params_file{ "executables/factor_graphs/mj_friction_map_eval.yaml" };
  param_loader params(params_file, argc, argv);

  init_random(params["random_seed"].as<int>());

  std::string model_filename = params["model_file"].as<std::string>();

  const bool visualize{ params["visualize"].as<bool>() };
  const std::string friction_map_file{ params["friction_map_file"].as<std::string>() };

  std::shared_ptr<mujoco_simulator_t> sim = std::make_shared<mujoco_simulator_t>(model_filename, visualize);

  std::shared_ptr<prx::mujoco_plant_t> mj_plant = std::make_shared<prx::mujoco_plant_t>("mujoco_plant");

  mj_plant->initialize(sim);
  prx::mujoco::mj_friction_plant_t::add_friction_to_plant(sim, mj_plant);

  sim->init_simulator(mj_plant);

  auto context = sim->get_context("mujoco");
  const auto sg = context.first;
  const auto ss = sg->get_state_space();
  const auto cs = sg->get_control_space();
  const auto ps = sg->get_parameter_space();
  prx_assert(ss != nullptr, "State space is null!!!");
  prx_assert(cs != nullptr, "Control space is null!!!");
  prx_assert(ps != nullptr, "Parameter space is null!!!");
  // const std::size_t ss_dim{ 27 };
  const std::size_t ss_dim{ ss->get_dimension() };
  const std::size_t cs_dim{ cs->get_dimension() };
  const auto ps_dim = ps->get_dimension();

  const std::vector<std::pair<double, double>> env_bounds{ std::make_pair(-XMAX, XMAX), std::make_pair(-YMAX, YMAX) };
  prx::regular_grid_t<friction_vector_t, 2> frictions_grid{ env_bounds, GRID_DIVISIONS };

  frictions_grid.from_file(params["friction_map_file"].as<>());

  friction_vector_t f;
  f = frictions_grid(-4.0, 4.0);
  PRX_DEBUG_VAR_1(f.transpose());
  // std::cout << "friction: " << frictions_grid(0.0, 4.0)[0] << std::endl;
  // std::cout << "friction: " << frictions_grid(-4.0, 0.0)[0] << std::endl;

  return 0;
}