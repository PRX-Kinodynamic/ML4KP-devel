#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/timer.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/planners/dirt_replan.hpp"
#include "prx/planning/planner_statistics.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/visualization/three_js_group.hpp"

#include <fstream>

using namespace prx;

int main(int argc, char* argv[])
{
    auto params = param_loader("examples/dynamic_obstacles/dynamic_obstacles_test.yaml");

    simulation_step = params["simulation_step"].as<double>();
    init_random(params["random_seed"].as<int>());

    std::string plant_name = params["/plant/name"].as<>();
    std::string plant_path = params["/plant/path"].as<>();
    auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
    prx_assert(plant != nullptr, "Plant is nullptr!");

    std::shared_ptr<world_model_t> sim(new world_model_t({plant},{}));
    sim -> create_context("dirt_context",{plant_name},{});
    auto context = sim -> get_context("dirt_context");
    auto cg = context.second;
    auto ss = context.first -> get_state_space();

    dirt_replan_t dirt(params["planner"].as<>());
    dirt_replan_specification_t dirt_spec(context.first,context.second);

    const double max_vel = 1.0;
    dirt_spec.h = [&](const space_point_t& s, const space_point_t& s2)
    {
        // Custom h function: ( eucledian distance from s to s2 ) / (max velocity)
        return space_t::euclidean_2d(s, s2) / max_vel;
    };

    // Two ways of accessing lengthy parameter paths
    int min_steps = params["/plant/min_steps"].as<int>();
    int max_steps = params["/plant/max_steps"].as<int>();

    dirt_spec.min_control_steps = min_steps;
    dirt_spec.max_control_steps = max_steps;
    dirt_spec.blossom_number    = params["blossom"].as<int>();
    dirt_spec.use_pruning       = params["pruning"].as<bool>();

    dirt_replan_query_t dirt_replan_query(context.first->get_state_space(),context.first->get_control_space());
    dirt_replan_query.start_state = context.first->get_state_space()->make_point();
    dirt_replan_query.goal_state  = context.first->get_state_space()->make_point();

    auto lower_bounds = params["/plant/state_space_lower_bound"].as<std::vector<double>>();
    auto upper_bounds = params["/plant/state_space_upper_bound"].as<std::vector<double>>();
    context.first -> get_state_space() -> set_bounds(lower_bounds, upper_bounds);

    context.first -> get_state_space() -> copy_point_from_vector(dirt_replan_query.start_state, params["/plant/start_state"].as<std::vector<double>>());
    context.first -> get_state_space() -> copy_point_from_vector(dirt_replan_query.goal_state, params["/plant/goal_state"].as<std::vector<double>>());
    
    dirt_replan_query.goal_region_radius = params["goal_region_radius"].as<double>();
    dirt_replan_query.get_visualization = true;

    condition_check_t checker(params["checker_type"].as<>(), params["checker_value"].as<double>()); 
    dirt.reset();
    dirt_replan_query.clear_outputs();

    dirt.link_and_setup_spec(&dirt_spec);
    dirt.preprocess();
    dirt.link_and_setup_query(&dirt_replan_query);
    dirt.resolve_query(&checker);

    dirt.fulfill_query();

    std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();
    three_js_group_t* vis_group = new three_js_group_t({plant});
    vis_group -> add_vis_infos(info_geometry_t::LINE, dirt_replan_query.tree_visualization, body_name, ss);
    vis_group -> output_html("output_original.html");
    delete vis_group;

    node_index_t sv = dirt.get_node_index();
    node_index_t rv = dirt.get_node_index(false);
    std::cout << sv << " " << rv << std::endl;
    dirt.prune_tree(sv,rv,true);
    dirt_replan_query.clear_outputs();
    dirt.fulfill_query();

    three_js_group_t* vis_group_2 = new three_js_group_t({plant});
    vis_group_2 -> add_vis_infos(info_geometry_t::LINE, dirt_replan_query.tree_visualization, body_name, ss);
    vis_group_2 -> output_html("output_pruned.html");
    delete vis_group_2;
    
}