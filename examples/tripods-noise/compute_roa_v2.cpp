#include <iostream>
#include <fstream>

#include "prx/planning/condition_check.hpp"
#include "prx/planning/planner_functions/planner_functions.hpp"
#include "prx/planning/noisy_world_model.hpp"
#include "prx/planning/world_model.hpp"

#include "prx/simulation/controllers/lqr.hpp"
#include "prx/simulation/controllers/noisy_controller.hpp"
#include "prx/simulation/controllers/torch_controller.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/simulation/multivalued_map/systems.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/simulation/plants/types/noisy_plant.hpp"

#include "prx/utilities/data_structures/gnn.hpp"
#include "prx/utilities/data_structures/tree.hpp"
#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/noise.hpp"
#include "prx/utilities/general/param_loader.hpp"

#include "prx/visualization/three_js_group.hpp"
using namespace prx;
int main(int argc, char* argv[])
{
  param_loader params = param_loader("examples/tripods/compute_roa.yaml", argc, argv);
  // params.print();

  simulation_step = params["simulation_step"].as<double>();
  prx::init_random(params["random_seed"].as<int>());
  const std::string system_name{ params["system_name"].as<>() };

  const double step_inc{ params["state_increment"].as<double>() };

  auto lower_bounds = params["/plant/starting_lower_bound"].as<std::vector<double>>();
  auto upper_bounds = params["/plant/ending_upper_bound"].as<std::vector<double>>();

  std::string plant_name = params["/plant/name"].as<>();
  std::string plant_path = params["/plant/path"].as<>();
  auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
  prx_assert(plant != nullptr, "Plant is nullptr!");

  world_model_t* world_model = new world_model_t({ plant }, {});
  world_model->create_context("context", { plant_name }, {});
  auto context = world_model->get_context("context");

  auto sg = context.first;
  space_t* ss = sg->get_state_space();
  std::size_t dimension{ ss->get_dimension() };

  prx::simulation::time_map_controllers_t::print_systems();
  trajectory_t traj(sg->get_state_space());
  prx::simulation::time_map_t tm(system_name, plant, sg);
  tm.set_duration(5);

  space_point_t state = ss->make_point();
  ss->copy(state, lower_bounds);
  do
  {
    tm(state, traj);
    for (std::size_t i = 0; i < traj.size(); ++i)
    {
      if (i > 0 && space_t::euclidean_2d(traj[i - 1], traj[i], 0, dimension) > 0.5)  // if angle wrap
      {
        std::cout << "\n";
      }
      std::cout << traj[i] << "\n";
    }
    std::cout << "\n";
  } while (state_space_step(*state, std::vector<double>(dimension, 0.5), dimension, lower_bounds, upper_bounds));
}
