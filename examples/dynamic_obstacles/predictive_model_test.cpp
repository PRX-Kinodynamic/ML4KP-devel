#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/planners/dirt_replan.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"

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

    auto obstacles = load_obstacles(params["environment"].as<>());
    std::vector<std::shared_ptr<movable_object_t>> obstacle_list = obstacles.second;
    std::vector<std::string> obstacle_names = obstacles.first;
        
    std::string plant_name = params["/plant/name"].as<>();
    std::string plant_path = params["/plant/path"].as<>();
    auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
    prx_assert(plant != nullptr, "Plant is nullptr!");

    std::shared_ptr<world_model_t> sim(new world_model_t({plant},{obstacle_list}));
    sensor_ptr_t sensor(new sensor_t("simple_sensor"));
    sim -> link_sensor(sensor);
    sim -> create_context("dirt_context",{plant_name},{obstacle_names});
    auto context = sim -> get_context("dirt_context");
    auto ss = context.first -> get_state_space();

    dirt_replan_t dirt(params["planner"].as<>());
    dirt_replan_specification_t dirt_spec(context.first,context.second);

    if (!fs::exists(out_path + params["output_dir"].as<std::string>()))
    {
        fs::create_directory(out_path + params["output_dir"].as<std::string>());
    }

    double t_plan = params["planning_time"].as<double>();
    // double t_wait = params["buffer_time"].as<double>();
    double t_H    = params["horizon"].as<double>();
    double full_time = params["max_replanning_cycles"].as<double>();

    std::string pred_fname = out_path + params["output_dir"].as<std::string>() + "/predictions_" +
        params["planning_time"].as<>() + "_" + params["horizon"].as<>() + ".txt";
    std::string gt_fname = out_path + params["output_dir"].as<std::string>() + "/ground_truth_" +
        params["planning_time"].as<>() + "_" + params["horizon"].as<>() + ".txt";
    
    std::ofstream pred_file, gt_file;
    pred_file.open(pred_fname);
    gt_file.open(gt_fname);
    

    std::unordered_map<std::string, std::vector<double>> current_poses;

    for (double current_time = 0; current_time <= full_time; current_time += t_H)
    {
        sim -> update_all_obstacle_poses(current_time);
        for (double step_time = current_time; step_time < current_time + t_H; step_time += t_plan)
        {
            current_poses = sensor -> get_obstacle_poses(step_time);
            pred_file << step_time;
            for (auto it : current_poses)
            {
                pred_file << "," << it.second.at(0) << "," << it.second.at(1);
            }
            pred_file << std::endl;
        }
        for (double step_time = current_time; step_time < current_time + t_H; step_time += t_plan)
        {
            sim -> update_all_obstacle_poses(step_time);
            current_poses = sensor -> get_obstacle_poses(step_time);
            gt_file << step_time;
            for (auto it : current_poses)
            {
                gt_file << "," << it.second.at(0) << "," << it.second.at(1);
            }
            gt_file << std::endl;
        }
    }
    pred_file.close();
    gt_file.close();
}