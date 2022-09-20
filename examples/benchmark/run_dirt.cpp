#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/dirt.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"

#include <fstream>

using namespace prx;

int main(int argc, char* argv[])
{
    auto params = param_loader("examples/benchmark/run_dirt.yaml", argc, argv);

    simulation_step = params["simulation_step"].as<double>();
    init_random(params["random_seed"].as<int>());

    auto obstacles = load_obstacles(params["environment"].as<>());
    std::vector<std::shared_ptr<movable_object_t>> obstacle_list = obstacles.second;
    std::vector<std::string> obstacle_names = obstacles.first;

    std::string plant_name = params["/plant/name"].as<>();
    std::string plant_path = params["/plant/path"].as<>();
    auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
    prx_assert(plant != nullptr, "Plant is nullptr!");

    world_model_t world_model({plant},{obstacle_list});
    world_model.create_context("dirt_context",{plant_name},{obstacle_names});
    auto context = world_model.get_context("dirt_context");

    dirt_t dirt(params["planner"].as<>());
    dirt_specification_t dirt_spec(context.first,context.second);

    dirt_spec.min_control_steps = params["/plant/min_steps"].as<int>();
    dirt_spec.max_control_steps = params["/plant/max_steps"].as<int>();
    dirt_spec.blossom_number = params["blossom"].as<int>();
    dirt_spec.use_pruning = params["pruning"].as<bool>();

    dirt_query_t dirt_query(context.first->get_state_space(),context.first->get_control_space());
    dirt_query.start_state = context.first->get_state_space()->make_point();
    dirt_query.goal_state  = context.first->get_state_space()->make_point();

    auto lower_bounds = params["/plant/state_space_lower_bound"].as<std::vector<double>>();
    auto upper_bounds = params["/plant/state_space_upper_bound"].as<std::vector<double>>();
    context.first -> get_state_space() -> set_bounds(lower_bounds, upper_bounds);

    context.first -> get_state_space() -> copy_point_from_vector(dirt_query.start_state, params["/plant/start_state"].as<std::vector<double>>());
    context.first -> get_state_space() -> copy_point_from_vector(dirt_query.goal_state, params["/plant/goal_state"].as<std::vector<double>>());
    
    dirt_query.goal_region_radius = params["goal_region_radius"].as<double>();
    dirt_query.get_visualization = params["visualize"].as<bool>();

    dirt.link_and_setup_spec(&dirt_spec);
    dirt.preprocess();
    dirt.link_and_setup_query(&dirt_query);

    condition_check_t checker(params["checker_type"].as<>(), params["checker_value"].as<int>()); //'

    dirt.resolve_query(&checker);
    dirt.fulfill_query(); 

    three_js_group_t* vis_group = new three_js_group_t({plant},{obstacle_list});
    std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();
    auto ss = context.first -> get_state_space();
    vis_group -> add_vis_infos(info_geometry_t::LINE, dirt_query.tree_visualization, body_name, ss);
    vis_group -> add_detailed_vis_infos(info_geometry_t::FULL_LINE, dirt_query.solution_traj, body_name, ss);
    vis_group -> add_animation(dirt_query.solution_traj, ss, dirt_query.start_state);
    vis_group -> output_html("output.html");
    delete vis_group;
}