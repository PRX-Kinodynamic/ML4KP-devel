#include <fstream>
#include <iostream>
#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/rrt_star.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/planners/planner.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"

using Transform = Eigen::Transform<double, 3, Eigen::Isometry>;
using prx::split;
using prx::utilities::convert_to;



int main(int argc, char* argv[]) {
    param_loader params("executables/peg_in_hole.yaml", argc, argv);

    PairNameObstacles obstacles{ load_obstacles(params["environment"].as<std::string>()) };
    const std::vector<std::shared_ptr<movable_object_t>> obstacle_list{ obstacles.second };
    const std::vector<std::string> obstacle_names{ obstacles.first };

    const std::string plant_name{ params["/plant/name"].as<std::string>() };
    const std::string plant_path{ params["/plant/path"].as<std::string>() };
    system_ptr_t plant{ system_factory_t::create_system(plant_name, plant_path) };
    prx_assert(plant != nullptr, "Plant is nullptr!");

    world_model_t world_model({ plant }, { obstacle_list });
    world_model.create_context("context", { plant_name }, { obstacle_names });
    auto context = world_model.get_context("context");
    std::shared_ptr<system_group_t> sys_group{ context.first };

    space_t* ss{ sys_group->get_state_space() };
    space_t* cs{ sys_group->get_control_space() };

    rrt_star_t rrt_star(params["/planner/name"].as<std::string>());
    rrt_star_specification_t rrt_star_spec(context.first, context.second);

    const std::string out_dir{ params["/out/dir"].as<std::string>() };
    const std::string file_prefix{ params["/out/file_prefix"].as<std::string>() };
    rrt_star.from_files(file_prefix, out_dir);

    Transform pegCenter_pegBottom{ Transform::Identity() };
    pegCenter_pegBottom.translation() = Eigen::Vector3d(0, 0, 25);

    // Apply transformation to each node in the tree
    // Note: The following loop is a placeholder and should be adapted
    for (auto& node : rrt_star.tree_visualization) {
        Transform node_transform;
        node_transform.translation() = node.state.get_position();
        node_transform.linear() = node.state.get_orientation().toRotationMatrix();

        Transform transformed_node = node_transform * pegCenter_pegBottom.inverse();

        node.state.set_position(transformed_node.translation());
        node.state.set_orientation(Eigen::Quaterniond(transformed_node.rotation()));
    }

    // Visualization
    three_js_group_t* vis_group = new three_js_group_t({ plant }, { obstacle_list });

    std::vector<double> floor_position = {0, 0, -1};
    std::vector<double> floor_orientation = {1, 0, 0, 0};
    std::vector<double> floor_size = {500, 500};
    std::string floor_color = "0xCCCCCC";
    vis_group->set_floor_plane(floor_position, floor_orientation, floor_size, floor_color);

    std::string body_name = plant_name + "/" + params["/plant/vis_body"].as<std::string>();
    vis_group->add_vis_infos(prx::info_geometry_t::LINE, rrt_star.tree_visualization, body_name, ss);

    std::vector<double> goal_state_vec = params["/plant/goal_state"].as<std::vector<double>>();
    std::vector<double> goal_state_position = { goal_state_vec[0], goal_state_vec[1], goal_state_vec[2] };
    std::string goal_color = "0xFF0000";
    double goal_radius = 0.5;
    vis_group->add_vis_infos(prx::info_geometry_t::SPHERE, goal_state_position, goal_color, goal_radius);

    vis_group->output_html("visualized_tree.html");

    delete vis_group;

    std::cout << "Visualization complete" << std::endl;
    return 0;
}