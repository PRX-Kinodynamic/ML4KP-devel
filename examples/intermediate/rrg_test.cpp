#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/rrg.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/planners/planner.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"

#include <fstream>

using namespace prx;

int main(int argc, char* argv[])
{
    auto params = param_loader("examples/basic/aorrt.yaml", argc, argv);

PRX_DEBUG_PRINT
    simulation_step = params["simulation_step"].as<double>();
    init_random(params["random_seed"].as<int>());
PRX_DEBUG_PRINT

    auto obstacles = load_obstacles(params["environment"].as<>());
    std::vector<std::shared_ptr<movable_object_t>> obstacle_list = obstacles.second;
    std::vector<std::string> obstacle_names = obstacles.first;
PRX_DEBUG_PRINT
        
    std::string plant_name = params["/plant/name"].as<>();
    std::string plant_path = params["/plant/path"].as<>();
    auto system = prx::system_factory_t::create_system(plant_name, plant_path);
    auto plant = std::dynamic_pointer_cast<omnirobot_FO_t>(system);
PRX_DEBUG_PRINT

    prx_assert(plant != nullptr, "Plant is nullptr!");

    world_model_t<> world_model({plant},{obstacle_list});
    world_model.create_context("rrg_context",{plant_name},{obstacle_names});
    auto context = world_model.get_context("rrg_context");

PRX_DEBUG_PRINT
    rrg_t rrg_star("RRG");
    rrg_specification_t rrg_spec(context.first,context.second);
PRX_DEBUG_PRINT

    auto cs = context.first -> get_control_space();

PRX_DEBUG_PRINT
        // Two ways of accessing lengthy parameter paths
    int min_steps = params["plant"]["min_steps"].as<int>();
    int max_steps = params["/plant/max_steps"].as<int>();

PRX_DEBUG_PRINT
    rrg_spec.steer = [&](space_point_t& state, space_point_t& local_goal, plan_t& plan, trajectory_t& traj)
    {
    // bool omnirobot_FO_t::
        return plant -> connect_points(state, local_goal, plan, traj);
    };
    // rrg_spec.valid_state = [](space_point_t& s)
    // {
        // Custom valid_state can be added here.  
    // };

    // rrg_spec.valid_check = [&rrg_spec](trajectory_t& traj)
    // {
        // Custom valid_check goes here...
        // Basically for x in traj, call valid_state
    // };

    // rrg_spec.sample_plan = [&](plan_t& plan, space_point_t pose)
    // {
        // Add custom sample plan here
    // };
    
PRX_DEBUG_PRINT
    rrg_spec.min_control_steps = min_steps;
    rrg_spec.max_control_steps = max_steps;

PRX_DEBUG_PRINT
    rrg_query_t rrg_query(context.first -> get_state_space(), context.first -> get_control_space());
    rrg_query.start_state = context.first->get_state_space() -> make_point();
    rrg_query.goal_state  = context.first->get_state_space() -> make_point();

PRX_DEBUG_PRINT
    auto lower_bounds = params["/plant/state_space_lower_bound"].as<std::vector<double>>();
    auto upper_bounds = params["/plant/state_space_upper_bound"].as<std::vector<double>>();
    context.first -> get_state_space() -> set_bounds(lower_bounds, upper_bounds);

PRX_DEBUG_PRINT
    context.first -> get_state_space() -> copy_point_from_vector(rrg_query.start_state, params["/plant/start_state"].as<std::vector<double>>());
    context.first -> get_state_space() -> copy_point_from_vector(rrg_query.goal_state, params["/plant/goal_state"].as<std::vector<double>>());

    rrg_query.goal_region_radius = params["goal_region_radius"].as<double>();
    
    // Alternatively, change the goal_check function
    // rrg_query.goal_check = [&](space_point_t pt)
    // {
    //    // Default is:
        // return space_t::euclidean_2d(pt, rrg_query.goal_state) < goal_region_radius;
    // }

PRX_DEBUG_PRINT
    rrg_query.get_visualization = params["visualize"].as<bool>();

PRX_DEBUG_PRINT
    rrg_star.link_and_setup_spec(&rrg_spec);
PRX_DEBUG_PRINT
    rrg_star.preprocess();
PRX_DEBUG_PRINT
    rrg_star.link_and_setup_query(&rrg_query);
PRX_DEBUG_PRINT

    condition_check_t checker(params["checker_type"].as<>(), params["checker_value"].as<int>()); //'
PRX_DEBUG_PRINT

    rrg_star.resolve_query(&checker);
PRX_DEBUG_PRINT
    rrg_star.fulfill_query(); 

    params.print();
        
        // TODO: Add function to visualization to replace tree_to_txt
        // tree_to_txt(dirt_query);
        
    three_js_group_t* vis_group = new three_js_group_t({plant},{obstacle_list});

    std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();
    auto ss = context.first -> get_state_space();

    vis_group -> add_vis_infos(info_geometry_t::LINE, rrg_query.tree_visualization, 
        body_name, ss);

    // vis_group -> add_detailed_vis_infos(info_geometry_t::FULL_LINE, rrg_query.solution_traj, 
    //     body_name, ss);

    // vis_group -> add_animation(rrg_query.solution_traj, ss, rrg_query.start_state);

    vis_group -> output_html("rrg_output.html");
    vis_group -> output_graph_to_csv("rrg_graph.csv", " ");
    
    delete vis_group;

    std::cout<<"End of program"<<std::endl;
}
