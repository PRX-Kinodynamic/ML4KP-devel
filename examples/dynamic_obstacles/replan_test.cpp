#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/planners/dirt_replan.hpp"
#include "prx/planning/replanners/replanner.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/visualization/three_js_group.hpp"

#include <fstream>

using namespace prx;

int main(int argc, char* argv[])
{
    auto params = param_loader("examples/dynamic_obstacles/replan_test.yaml");

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
    sim -> create_context("dirt_context",{plant_name},{obstacle_names});
    auto context = sim -> get_context("dirt_context");
    auto ss = context.first -> get_state_space();

    dirt_replan_t dirt(params["planner"].as<>());
    dirt_replan_specification_t dirt_spec(context.first,context.second);
    replanner_t replanner(params["planner"].as<>());
    replanner.setup(params);

    dirt_spec.h = [&](const space_point_t& s, const space_point_t& s2)
    {
        // Custom h function: ( eucledian distance from s to s2 ) / (max velocity)
        return space_t::euclidean_2d(s, s2)/1.0;
    };

    dirt_spec.min_control_steps = params["/plant/min_steps"].as<int>();
    dirt_spec.max_control_steps = params["/plant/max_steps"].as<int>();
    dirt_spec.blossom_number    = params["blossom"].as<int>();
    dirt_spec.use_pruning       = params["pruning"].as<bool>();

    dirt_replan_query_t dirt_query(context.first->get_state_space(),context.first->get_control_space());
    dirt_query.start_state = ss->make_point();
    dirt_query.goal_state  = ss->make_point();

    auto lower_bounds = params["/plant/state_space_lower_bound"].as<std::vector<double>>();
    auto upper_bounds = params["/plant/state_space_upper_bound"].as<std::vector<double>>();
    ss -> set_bounds(lower_bounds, upper_bounds);

    ss -> copy_point_from_vector(dirt_query.goal_state, params["/plant/goal_state"].as<std::vector<double>>());
    
    dirt_query.goal_region_radius = params["goal_radius"].as<double>();
    dirt_query.get_visualization = true;

    three_js_group_t* vis_group = new three_js_group_t({plant},{});
    std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();

    replanner.link_planner(&dirt,&dirt_spec,&dirt_query);
    replanner.link_world_model(sim);
    PRX_DEBUG_PRINT
    
    for (int j = 0; j < params["num_trials"].as<int>(); j++)
    {
        ss -> copy_point_from_vector(dirt_query.start_state, params["/plant/start_state"].as<std::vector<double>>());
        dirt_query.start_time = 0;

        replanner.reset();
        replanner.resolve_query();

        std::string fname = out_path + params["output_dir"].as<std::string>() + "/" +
                    "trajectory_" + std::to_string(j) +".txt";
        std::ofstream fout;
        fout.open(fname);
        for (unsigned i = 0; i < replanner.full_solution_trajectory -> size(); i++)
        {
            sim -> update_all_obstacle_poses(i*simulation_step);
            auto step_state = replanner.full_solution_trajectory -> at(i);
            fout << ss -> print_point(step_state,4) 
            << "," << dirt_spec.valid_state(step_state) << std::endl;
        }
        fout.close();
        output_progress_bar(1.0 * j/params["num_trials"].as<int>());
    }
    
    /*
    for (int i = 0; i < max_cycles && continue_planning; i++)
    {
        std::cout << "Replanning iteration " << i << std::endl;
        if (i == 0) 
        {
            ss -> copy_point_from_vector(dirt_query.start_state, params["/plant/start_state"].as<std::vector<double>>());
            dirt_query.start_time = 0;
        }

        dirt_spec.replanning_cycle = (i + 1) * params["replanning_cycle"].as<double>();

        dirt.reset();
        checker.reset();
        dirt_query.clear_outputs();

        dirt.link_and_setup_spec(&dirt_spec);
        dirt.preprocess();
        dirt.link_and_setup_query(&dirt_query);
        
        dirt.resolve_query(&checker);
        dirt.fulfill_query();

        PRX_DEBUG_PRINT
        std::cout << "Solution cost: " << dirt_query.solution_cost << std::endl;
        if (dirt_query.solution_cost == 0) 
        {
            continue_planning = false;
            break;
        }

        vis_group -> add_vis_infos(info_geometry_t::LINE, dirt_query.tree_visualization, body_name, ss);
        full_traj += dirt_query.solution_traj;
        auto last_state = full_traj.back();
        continue_planning &= !dirt_query.goal_check(last_state);
        if (i != max_cycles - 1 && !dirt_query.goal_check(last_state))
        {
            full_traj.resize(full_traj.size()-1);
        }

        bool valid = true;
        for (unsigned i = 0; i < dirt_query.solution_traj.size(); i++)
        {
            sim -> update_all_obstacle_poses(dirt_query.start_time + i*simulation_step);
            auto step_state = dirt_query.solution_traj.at(i);
            valid &= dirt_spec.valid_state(step_state);
        }

        continue_planning &= valid;
        ss -> copy_point(dirt_query.start_state, dirt_query.solution_traj.back());
        dirt_query.start_time += dirt_query.solution_cost;
        std::cout << "Resetting obstacles to time " << dirt_query.start_time - params["checker_value"].as<double>() << std::endl;
        sim -> update_all_obstacle_poses(dirt_query.start_time - params["checker_value"].as<double>());
    }

    

    vis_group -> add_detailed_vis_infos(info_geometry_t::FULL_LINE, full_traj, body_name, ss);
    vis_group -> add_animation(dirt_query.solution_traj, ss, dirt_query.start_state);
    vis_group -> output_html("output.html");

    delete vis_group;
    */
}