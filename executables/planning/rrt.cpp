#include <fstream>
#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/aorrt.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/planners/planner.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/planning/planner_functions/tree_fix_time_discretization.hpp"

#include "prx/planning/planners/rrt_v2.hpp"
#include "prx/simulation/plants/SE2_holonomic_system.hpp"
int main(int argc, char* argv[])
{
  prx::param_loader params(argc, argv);
  prx::param_loader environment_params(params["environment"].as<>());

  using RRTMemory = prx::planners::rrt_memory_t<prx::SE2_holonomic_system_t>;
  using RRTFunctions = prx::planners::rrt_functions_t<prx::SE2_holonomic_system_t>;

  prx::planners::motion_planner_t<RRTMemory, RRTFunctions> rrt(params, environment_params);

  std::cout << "End of program" << std::endl;
}
