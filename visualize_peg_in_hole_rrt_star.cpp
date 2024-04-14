#include <fstream>
#include <sstream>
#include <vector>
#include <iostream>
#include <memory>

#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/rrt_star.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/planners/planner.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"

int main(int argc, char* argv[])
{
    prx::param_loader params("executables/peg_in_hole.yaml", argc, argv);

    prx::simulation_step = params["simulation_step"].as<double>();
    prx::init_random(params["random_seed"].as<int>());

    prx::PairNameObstacles obstacles{ prx::load_obstacles(params["environment"].as<std::string>()) };
    const std::vector<std::shared_ptr<prx::movable_object_t>> obstacle_list{ obstacles.second };
    const std::vector<std::string> obstacle_names{ obstacles.first };

    const std::string plant_name{ params["/plant/name"].as<std::string>() };
    const std::string plant_path{ params["/plant/path"].as<std::string>() };
    prx::system_ptr_t plant{ prx::system_factory_t::create_system(plant_name, plant_path) };
    prx_assert(plant != nullptr, "Plant is nullptr!");

    prx::world_model_t world_model({ plant }, { obstacle_list });
    world_model.create_context("context", { plant_name }, { obstacle_names });
    auto context = world_model.get_context("context");
    std::shared_ptr<prx::system_group_t> sys_group{ context.first };

    prx::space_t* ss{ sys_group->get_state_space() };
    prx::space_t* cs{ sys_group->get_control_space() };
    prx::space_t* ps{ sys_group->get_parameter_space() };

    // Load trajectory from the file
    std::string trajectory_file_path = "/home/common/im316/CORL/algorithms/learned_RRTStar_5.txt"; // Update the path to your file
    std::ifstream file(trajectory_file_path);
    if (!file.is_open()) {
        std::cerr << "Failed to open trajectory file: " << trajectory_file_path << std::endl;
        return 1;
    }

    std::string line;
    prx::trajectory_t loaded_trajectory(ss);
    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        std::vector<double> point;
        double value;
        while (iss >> value)
        {
            point.push_back(value);
        }
        loaded_trajectory.copy_onto_back(point.data());
    }

    // Visualization
    prx::three_js_group_t* vis_group = new prx::three_js_group_t({ plant }, { obstacle_list });

    std::string body_name = params["/plant/name"].as<std::string>() + "/" + params["/plant/vis_body"].as<std::string>();

    //vis_group->set_floor_plane(std::vector<double>({ 0, 0, -3 }), std::vector<double>({ 0.707, 0, 0, 0.707 }), std::vector<double>({ 500, 500 }), "0xbbbbbb");
    vis_group->set_floor_plane(std::vector<double>({ 0.13, 4.356e-11, 1.04 }), std::vector<double>({ 0.0, 0.0, 0.0, 1.0 }), std::vector<double>({ 500, 500 }), "0xbbbbbb");
    vis_group->add_detailed_vis_infos(prx::info_geometry_t::FULL_LINE, loaded_trajectory, body_name, ss);
    //vis_group->add_animation(loaded_trajectory, ss, loaded_trajectory[0]);
    vis_group->add_animation(loaded_trajectory, ss, loaded_trajectory.operator[](0.0));
    vis_group->output_html("peg_in_hole_solution_traj.html");

    delete vis_group;

    std::cout << "End of program" << std::endl;
    return 0;
}

