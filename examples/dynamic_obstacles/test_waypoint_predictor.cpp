#include "prx/utilities/defs.hpp"
#include "prx/utilities/learned_modules/learned_modules_utils.hpp"
#include "prx/utilities/learned_modules/waypoint_predictor.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/planners/dirt.hpp"
#include "prx/simulation/loaders/dynamic_obstacle_loader.hpp"

using namespace prx;

int main(int argc, char* argv[])
{
    std::string params_file;
    if (argc <= 1)
    {
        params_file = "examples/dynamic_obstacles/test_waypoint_predictor.yaml";
        // prx_throw("The planner evaluation executable needs a parameter file!");
    }
    else 
    {
        params_file = std::string(argv[1]);
    }

    param_loader params(params_file);
    simulation_step = params["simulation_step"].as<double>();
    init_random(params["random_seed"].as<int>());

    waypoint_predictor_t waypoint_predictor(params);

    std::string plant_name = params["/plant/name"].as<>();
    std::string plant_path = params["/plant/path"].as<>();
    auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
    prx_assert(plant != nullptr, "Plant is nullptr!");

     auto obstacles = load_dynamic_obstacles(params["environment"].as<>());
    std::vector<std::shared_ptr<movable_object_t>> obstacle_list = obstacles.second;
    std::vector<std::string> obstacle_names = obstacles.first;

    std::shared_ptr<world_model_t> sim(new world_model_t({plant},{obstacle_list}));
    sim -> create_context("dirt_context",{plant_name},{obstacle_names});
    
    auto context = sim -> get_context("dirt_context");
    auto cg = context.second;
    auto ss = context.first -> get_state_space();

    dirt_t dirt("dirt");
    dirt_specification_t dirt_spec(context.first,context.second);
    dirt_spec.use_prescience = true;

    dirt_spec.time_valid_state = [&](space_point_t& s, double current_time)
    {
        // Custom time_valid_state function:
        ss -> copy_from_point(s);
        sim -> update_all_obstacle_poses(current_time);

        if(cg->in_collision() || !ss->satisfies_bounds(s))
		{
			return false;
		}
		return true;
    };

    double planning_time = params["planning_time"].as<double>();

    std::vector<double> current_state_vec = {-9,0,0,0,0};
    std::vector<double> current_obs;
    space_point_t current_state = ss->make_point();
    double max_time = params["max_time"].as<double>();

    for (double time = 0; time <= max_time; time++)
    {
        current_obs.clear();
        // TODO
        // auto current_obs_infos = sim->get_all_obstacle_poses(time,"box_");
        // for (auto obs : current_obs_infos)
        //     for(auto r : obs) 
        //         current_obs.push_back(r);
        auto result = waypoint_predictor.get_waypoint(current_state_vec, current_obs);
        while (result.size() < ss->get_dimension()) 
            result.push_back(0.0);
        ss -> copy_point_from_vector(current_state, result);
        std::cout << "Predicted next state: " << ss->print_point(current_state,4) << std::endl;
        if (!dirt_spec.time_valid_state(current_state, time))
        {
            std::cout << "Collision!" << std::endl;
            break;
        }
        current_state_vec.clear();
        ss -> copy_vector_from_point(current_state_vec, current_state);
    }
}