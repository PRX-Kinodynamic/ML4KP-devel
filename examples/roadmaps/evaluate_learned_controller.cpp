#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/dirt.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/simulation/controllers/learned_controller.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"

using namespace prx;

int main(int argc, char* argv[])
{
  auto params = param_loader("examples/roadmaps/trailer_car.yaml");

  simulation_step = params["simulation_step"].as<double>();
  init_random(params["random_seed"].as<int>());

  std::string plant_name = params["/plant/name"].as<>();
  std::string plant_path = params["/plant/path"].as<>();
  auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
  prx_assert(plant != nullptr, "Plant is nullptr!");

  world_model_t world_model({ plant }, {});
  world_model.create_context("dirt_context", { plant_name }, {});
  auto context = world_model.get_context("dirt_context");

  dirt_specification_t dirt_spec(context.first, context.second);
  auto learned_controller_params = param_loader(params["controller"].as<std::string>());
  learned_controller_t controller(plant, learned_controller_params);

  dirt_query_t dirt_query(context.first->get_state_space(), context.first->get_control_space());
  dirt_query.goal_state = context.first->get_state_space()->make_point();
  dirt_query.start_state = context.first->get_state_space()->make_point();
  dirt_query.goal_region_radius = params["goal_region_radius"].as<double>();
  dirt_query.goal_check = [&](space_point_t point) {
    double diff2 = (point->at(0) - dirt_query.goal_state->at(0)) * (point->at(0) - dirt_query.goal_state->at(0)) +
                   (point->at(1) - dirt_query.goal_state->at(1)) * (point->at(1) - dirt_query.goal_state->at(1));
    diff2 += norm_angle_pi(point->at(2) - dirt_query.goal_state->at(2)) *
             norm_angle_pi(point->at(2) - dirt_query.goal_state->at(2));
    return std::sqrt(diff2) < dirt_query.goal_region_radius;
  };

  const unsigned num_trials = int(1e4);
  unsigned num_successes = 0;
  space_point_t final_state = context.first->get_state_space()->make_point();
  for (int i = 0; i < num_trials; i++)
  {
    dirt_spec.sample_state(dirt_query.goal_state);
    controller.fulfill_query(dirt_spec, dirt_query);
    if (dirt_query.solution_traj.size() > 0)
    {
      plant->get_state_space()->copy_point(final_state, dirt_query.solution_traj.back());
      if (dirt_query.goal_check(final_state))
        num_successes++;
    }
    output_progress_bar(1.0 * i / num_trials);
  }
  std::cout << "Success rate: " << (double)num_successes / num_trials << std::endl;
}