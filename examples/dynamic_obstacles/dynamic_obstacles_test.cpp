#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/timer.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/planners/dirt.hpp"
#include "prx/planning/planner_statistics.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/simulation/loaders/dynamic_obstacle_loader.hpp"

#ifdef __cpp_lib_filesystem
    #include <filesystem.hpp>
    namespace fs = std::filesystem;
#else
    #define _LIBCPP_NO_EXPERIMENTAL_DEPRECATION_WARNING_FILESYSTEM
    #include <experimental/filesystem>
    namespace fs = std::experimental::filesystem;
#endif
#include <fstream>

using namespace prx;

int main(int argc, char* argv[])
{
    std::string params_file;
    if (argc <= 1)
    {
        params_file = "examples/dynamic_obstacles/replan_test.yaml";
        // prx_throw("The planner evaluation executable needs a parameter file!");
    }
    else 
    {
        params_file = std::string(argv[1]);
    }

    param_loader params(params_file);
    simulation_step = params["simulation_step"].as<double>();
    init_random(params["random_seed"].as<int>());

    // auto obstacles = load_obstacles(params["environment"].as<>());
    auto obstacles = load_dynamic_obstacles(params["environment"].as<>());
    std::vector<std::shared_ptr<movable_object_t>> obstacle_list = obstacles.second;
    std::vector<std::string> obstacle_names = obstacles.first;
        
    std::string plant_name = params["/plant/name"].as<>();
    std::string plant_path = params["/plant/path"].as<>();
    auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
    prx_assert(plant != nullptr, "Plant is nullptr!");

    std::shared_ptr<world_model_t> sim(new world_model_t({plant},{obstacle_list}));
    sensor_ptr_t sensor(new sensor_t("simple_sensor"));
    sim -> link_sensor(sensor);
    sensor -> print_obstacle_infos();
    sim -> create_context("dirt_context",{plant_name},{obstacle_names});
    auto context = sim -> get_context("dirt_context");
    auto cg = context.second;
    auto ss = context.first -> get_state_space();

    dirt_t dirt(params["planner"].as<>());
    dirt_specification_t dirt_spec(context.first,context.second);

    const double max_vel = 1.0;
    dirt_spec.h = [&](const space_point_t& s, const space_point_t& s2)
    {
        // Custom h function: ( eucledian distance from s to s2 ) / (max velocity)
        return space_t::euclidean_2d(s, s2, 0, 3) / max_vel;
    };

    dirt_spec.use_prescience = params["prescience"].as<bool>();
    std::unordered_map<std::string, std::vector<double>> poses;
    dirt_spec.time_valid_state = [&](space_point_t& s, double current_time)
    {
        // Custom time_valid_state function:
        ss -> copy_from_point(s);
        sim -> update_all_obstacle_poses(current_time);

        // The below part is redundant, but we gotta test it anyway
        // poses = sensor->get_obstacle_poses(current_time);
        // for (auto p : poses)
        // {
        //     sim -> update_obstacle_pose(p.first, p.second);
        // }

        // This is standard
        if(cg->in_collision() || !ss->satisfies_bounds(s))
		{
			return false;
		}
		return true;
    };

    dirt_spec.time_valid_trajectory = [&](trajectory_t& traj, double start_time)
    {
        for (unsigned i = 0; i < traj.size(); i++)
		{
			auto s = traj.at(i);
            if (!dirt_spec.time_valid_state(s, start_time + i * simulation_step))
            {
                return false;
            }
		}
        return true;
    };

    // Two ways of accessing lengthy parameter paths
    int min_steps = params["/plant/min_steps"].as<int>();
    int max_steps = params["/plant/max_steps"].as<int>();

    dirt_spec.min_control_steps = min_steps;
    dirt_spec.max_control_steps = max_steps;
    dirt_spec.blossom_number    = params["blossom"].as<int>();
    dirt_spec.use_pruning       = params["pruning"].as<bool>();

    dirt_query_t dirt_query(context.first->get_state_space(),context.first->get_control_space());
    dirt_query.start_state = context.first->get_state_space()->make_point();
    dirt_query.goal_state  = context.first->get_state_space()->make_point();

    auto lower_bounds = params["/plant/state_space_lower_bound"].as<std::vector<double>>();
    auto upper_bounds = params["/plant/state_space_upper_bound"].as<std::vector<double>>();
    context.first -> get_state_space() -> set_bounds(lower_bounds, upper_bounds);

    context.first -> get_state_space() -> copy_point_from_vector(dirt_query.start_state, params["/plant/start_state"].as<std::vector<double>>());
    context.first -> get_state_space() -> copy_point_from_vector(dirt_query.goal_state, params["/plant/goal_state"].as<std::vector<double>>());
    
    dirt_query.goal_region_radius = params["goal_region_radius"].as<double>();

    condition_check_t checker(params["checker_type"].as<>(), 0.1 * params["checker_value"].as<double>()); 
    int num_trials = params["num_trials"].as<int>();

    if (!fs::exists(out_path + params["output_dir"].as<std::string>()))
    {
        fs::create_directory(out_path + params["output_dir"].as<std::string>());
    }

    for (int i = 0; i < num_trials; i++)
    {
        dirt.reset();
        dirt_query.clear_outputs();

        dirt.link_and_setup_spec(&dirt_spec);
        dirt.preprocess();
        dirt.link_and_setup_query(&dirt_query);

        planner_statistics_t stats;
        stats.link_planner(&dirt);
        stats.link_criterion(&checker);
        stats.repeat_data_gathering(10,false);

        dirt.fulfill_query();

        std::string fname = out_path + params["output_dir"].as<std::string>() + "/" +
                    "dirt_" + std::to_string(i) + ".txt";
        std::cout << fname << std::endl;
        std::ofstream fout;
        fout.open(fname);
        fout << stats.serialize() << std::endl;
        fout.close();

        fname = out_path + params["output_dir"].as<std::string>() + "/" +
                    "trajectory_" + std::to_string(i) + ".txt";
        std::cout << fname << std::endl;
        fout.open(fname);

        for (unsigned i = 0; i < dirt_query.solution_traj.size(); i++)
        {
            sim -> update_all_obstacle_poses(i*simulation_step);
            auto step_state = dirt_query.solution_traj.at(i);
            fout << context.first -> get_state_space() -> print_point(step_state,4) 
            << "," << dirt_spec.valid_state(step_state) << "," << dirt_spec.time_valid_state(step_state,i*simulation_step) << std::endl;
        }
        fout.close();
    }
}