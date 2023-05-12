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

int main(int argc, char* argv[])
{
  auto params = param_loader("executables/tensegrity/tensegrity_dartmouth.yaml", argc, argv);
  init_random(params["random_seed"].as<int>());

  std::shared_ptr<mujoco_simulator_t> sim =
      std::make_shared<mujoco_simulator_t>("tensegrity/tensegrity_dartmouth_fixed.xml", params["visualize"].as<bool>());
  sim->init_simulator();

  auto context = sim->get_context("mujoco");
  auto ss = context.first->get_state_space();
  auto cs = context.first->get_control_space();
  std::shared_ptr<system_group_t> sg = context.first;

  prx::utilities::csv_reader_t reader(prx::out_path + "/tensegrity/states_0.txt");

  MujocoState state = sim->get_state();
  for (auto line : reader)
  {
    std::transform(line.begin(), line.end(),
                   state.qpos.begin(),  // write to the same location
                   [](const std::string& c) { return std::stod(c); });
    // PRX_DEBUG_VAR_2(line.size(), dbl_line.size());
    sim->set_state(state);
    // sim->step_simulation(propagate_step::FIRST_STEP);
    // ss->copy_from(dbl_line);
    // sim->forward_simulation();
  }
}