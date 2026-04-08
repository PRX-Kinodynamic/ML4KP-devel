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

int main(int argc, char* argv[])
{
  prx::param_loader params(argc, argv);
  prx::param_loader environment_params(params["environment"].as<>());

  prx::obstacle_loader_t obstacle_loader{ prx::obstacle_loader_t(environment_params) };
  auto obstacle_list = obstacle_loader.get_obstacles();

  prx::three_js_group_t* vis_group = new prx::three_js_group_t({}, { obstacle_list });
  vis_group->output_html("environment.html");

  delete vis_group;

  std::cout << "End of program" << std::endl;
}
