#ifndef TORCH_NOT_BUILT
#include "prx/utilities/defs.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/utilities/learned_modules/learned_controller.hpp"
#include "prx/planning/planners/dirt.hpp"

#ifdef __cpp_lib_filesystem
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;
#else
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#endif

using namespace prx;

int main(int argc, char* argv[])
{
  try
  {
    std::string params_file;
    if (argc <= 1)
    {
      params_file = "local_goal/car_like.yaml";
      // prx_throw("This executable needs a parameter file!");
    }
    else
    {
      params_file = std::string(argv[1]);
    }

    std::cout << "Using params file: " << params_file << std::endl;
    param_loader params(params_file);
    simulation_step = params["simulation_step"].as<double>();
    int random_seed = params["random_seed"].as<int>();
    init_random(random_seed);
    torch::set_num_threads(1);

    std::string plant_name = params["/plant/name"].as<std::string>();
    std::string plant_path = params["/plant/path"].as<std::string>();
    auto plant = system_factory_t::create_system(plant_name, plant_path);

    std::vector<double> lower_bounds = params["/plant/state_space_lower_bound"].as<std::vector<double>>();
    std::vector<double> upper_bounds = params["/plant/state_space_upper_bound"].as<std::vector<double>>();
    plant->set_state_space_bounds(lower_bounds, upper_bounds);

    world_model_t world_model({ plant }, {});
    world_model.create_context("planning_context", { plant_name }, {});
    auto context = world_model.get_context("planning_context");

    auto ss = context.first->get_state_space();
    auto cs = context.first->get_control_space();
    auto sg = context.first;

    dirt_t dirt("dirt");
    dirt_specification_t spec(context.first, context.second);
    spec.min_control_steps = params["/plant/min_steps"].as<int>();
    spec.max_control_steps = params["/plant/max_steps"].as<int>();

    dirt_query_t query(ss, cs);
    query.start_state = ss->make_point();
    query.goal_state = ss->make_point();
    query.get_visualization = true;

    learned_controller_t controller(params);

    spec.sample_state = [&](space_point_t s) {
      ss->sample(s);
      for (int i = 3; i < 5; i++)
        s->at(i) = 0;
    };

    spec.valid_check = [&](trajectory_t t) { return true; };

    query.goal_check = [&](space_point_t s) {
      double diff = (s->at(0) - query.goal_state->at(0)) * (s->at(0) - query.goal_state->at(0)) +
                    (s->at(1) - query.goal_state->at(1)) * (s->at(1) - query.goal_state->at(1));
      diff += norm_angle_pi(s->at(2) - query.goal_state->at(2)) * norm_angle_pi(s->at(2) - query.goal_state->at(2));
      return std::sqrt(diff) < query.goal_region_radius;
    };

    spec.sample_state(query.start_state);
    spec.sample_state(query.goal_state);
    std::cout << "Start: " << ss->print_point(query.start_state, 4) << std::endl;
    std::cout << "Goal: " << ss->print_point(query.goal_state, 4) << std::endl;
    controller.fulfill_query(query, spec);
    query.solution_traj.to_file(prx::out_path + "traj.txt");
    query.solution_plan.to_file(prx::out_path + "plan.txt");
    std::cout << query.solution_traj.size() << std::endl;

    ss->copy_point(query.start_state, query.goal_state);
    spec.sample_state(query.goal_state);
    std::cout << "Start: " << ss->print_point(query.start_state, 4) << std::endl;
    std::cout << "Goal: " << ss->print_point(query.goal_state, 4) << std::endl;
    controller.fulfill_query(query, spec);

    query.solution_traj.to_file(prx::out_path + "traj.txt", std::ofstream::app);
    query.solution_plan.to_file(prx::out_path + "plan.txt", std::ofstream::app);
    std::cout << query.solution_traj.size() << std::endl;
  }
  catch (const prx_assert_t& e)
  {
    std::cerr << e.what() << std::endl;
    return -1;
  }
}

#else
int main()
{
}
#endif