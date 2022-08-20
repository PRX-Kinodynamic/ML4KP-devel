#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/dirt.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/planners/planner.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"

#include <fstream>

#include "prx/planning/planner_statistics.hpp"
#include "prx/simulation/system_factory.hpp"
#include <math.h>



using namespace prx;

int main(int argc, char* argv[])
{
    auto params = param_loader("examples/basic/dirt.yaml", argc, argv);

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

    // dirt_spec.valid_state = [](space_point_t& s)
    // {
        // Custom valid_state can be added here.  
    // };

    // dirt_spec.valid_check = [&dirt_spec](trajectory_t& traj)
    // {
        // Custom valid_check goes here...
        // Basically for x in traj, call valid_state
    // };
	
    //params["max_speed"].set(system_factory_t::get_system_max_velocity(plant_name, plant));
    
    
    const double max_vel = params["max_speed"].as<double>();
    
    
  
    dirt_spec.h = [&](const space_point_t& s, const space_point_t& s2)
    {
        // Custom h function: ( eucledian distance from s to s2 ) / (max velocity)
        return space_t::euclidean_2d(s, s2) / max_vel;
    };

    // Two ways of accessing lengthy parameter paths
    int min_steps = params["plant"]["min_steps"].as<int>();
    int max_steps = params["/plant/max_steps"].as<int>();

    // dirt_spec.sample_plan = [&](plan_t& plan, space_point_t pose)
    // {
        // Add custom sample plan here
    // };

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
    // Alternatively, change the goal_check function
    // rrt_query.goal_check = [&](space_point_t pt)
    // {
    //    // Default is:
        // return space_t::euclidean_2d(pt, rrt_query.goal_state) < goal_region_radius;
    // }
    
    dirt_spec.valid_state = [&,dirt_spec](space_point_t a)
        {
            auto t0 = a -> at(2);
            auto t1 = a -> at(3);
            
            auto TAU = 2*M_PI;
            auto ang = t1 - t0;
 	    auto ans = fmod(( fmod((ang+M_PI), TAU) + TAU), TAU) - M_PI;
            
            /*auto TAU = 2*M_PI;
	    auto ans = 0.0;
	    
	    auto aa = fmod((t0 - t1), TAU);
	    auto bb = fmod((t1 - t0), TAU);
	    
	    if (aa<bb){
	    	ans = -aa;}
    
	    else{
	    	ans = bb;}*/
            
            //return( (default_valid_state(a, context.first->get_state_space(), context.second)) && ((M_PI - fabs(fmod(fabs(t1 - t0), 2*M_PI) - M_PI)) < M_PI/4) && ((M_PI - fabs(fmod(fabs(t1 - t0), 2*M_PI) - M_PI)) > -(M_PI/4)) );
            
            return( (default_valid_state(a, context.first->get_state_space(), context.second)) && (ans< M_PI/4) && (ans > -(M_PI/4)) );
             

 
        };
    
    dirt_query.get_visualization = params["visualize"].as<bool>();

    dirt.link_and_setup_spec(&dirt_spec);
    dirt.preprocess();
    dirt.link_and_setup_query(&dirt_query);

    //condition_check_t checker(params["checker_type"].as<>(), params["checker_value"].as<int>()); //'
    condition_check_t checker(params["checker_type"].as<>(), params["checker_value"].as<double>()/600.0); 
    
    
    std::ofstream fout;
    //For statistics
    int stats_runs = 10; //params["planner_runs"].as<int>();
		for (int i = 0; i < stats_runs; i++)
		{				
			PRX_DEBUG_PRINT
			dirt.link_and_setup_spec(&dirt_spec);
			PRX_DEBUG_PRINT
			dirt.preprocess();
			PRX_DEBUG_PRINT
			dirt.link_and_setup_query(&dirt_query);
			PRX_DEBUG_PRINT
			
			PRX_DEBUG_PRINT
			planner_statistics_t stats;
			stats.link_planner(&dirt);
			stats.link_criterion(&checker);
			PRX_DEBUG_PRINT
			//stats.repeat_data_gathering(stats_iterations);
			stats.repeat_data_gathering(600);
			std::string full_filename = lib_path+params["data_output_folder"].as<std::string>()+params["planner_name"].as<std::string>()+"_"+std::to_string(i)+".txt";
			std::cout << full_filename << std::endl;
			fout.open(full_filename);
			fout<<stats.serialize() << std::endl;
			fout.close();
			
			
			

			//if(get_visualization)
			if(true)
			{
				dirt.fulfill_query();
				std::string vis_body1 = params["/plant/name"].as<>() + "/" + params["/plant/vis_body1"].as<>();
				std::string vis_body2 = params["/plant/name"].as<>() + "/" + params["/plant/vis_body2"].as<>();
				three_js_group_t* vis_group = new three_js_group_t({plant},{obstacle_list});

				
				for(auto& traj : dirt_query.tree_visualization)
				{
					vis_group->add_vis_infos(info_geometry_t::LINE, traj, vis_body1, context.first->get_state_space());
					vis_group->add_vis_infos(info_geometry_t::LINE, traj, vis_body2, context.first->get_state_space());
					
					
				}
				
				
			
			fout.close();

				if (dirt_query.solution_cost > 0)
				// if (dirt_query.solution_plan.duration() > 0)
				{
					vis_group->add_vis_infos(info_geometry_t::FULL_LINE, dirt_query.solution_traj, vis_body1, context.first->get_state_space(),"0x00ffff");
					
					vis_group->add_vis_infos(info_geometry_t::FULL_LINE, dirt_query.solution_traj, vis_body2, context.first->get_state_space(),"0xff0033");
					
					double timestamp=0;
					for(auto state : dirt_query.solution_traj)
					{
						context.first->get_state_space()->copy_from_point(state);
						vis_group->snapshot_state(timestamp);
						timestamp+=simulation_step;
					}
				std::string full_filename_traj = lib_path+params["data_output_folder"].as<std::string>()+params["planner_name"].as<std::string>()+"_traj_"+std::to_string(i)+".txt";
			std::cout << full_filename_traj << std::endl;
			fout.open(full_filename_traj);
			fout<< dirt_query.solution_traj.print() << std::endl;
			fout.close();
				}
				else
				{
					double timestamp=0;
					context.first->get_state_space()->copy_from_point(dirt_query.start_state);
					vis_group->snapshot_state(timestamp);
					timestamp+=simulation_step;
				}
				vis_group->output_html(params["output_html"].as<std::string>()+"_"+std::to_string(i)+".html");
				delete vis_group;
			}
			dirt_query.clear_outputs();
			dirt.reset();
		}
    
    
    
    
    
    
    
    
	
    //*dirt.resolve_query(&checker);
    //*dirt.fulfill_query(); 

    params.print();
        
    // TODO: Add function to visualization to replace tree_to_txt
    // tree_to_txt(dirt_query);
        
    //*three_js_group_t* vis_group = new three_js_group_t({plant},{obstacle_list});
    // TODO: Add function to visualization to replace tree_to_html
    // tree_to_html(vis_group, dirt_query, context.first -> get_state_space());

    //*std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();
    //*auto ss = context.first -> get_state_space();

    //*vis_group -> add_vis_infos(info_geometry_t::LINE, dirt_query.tree_visualization, body_name, ss);

    //*vis_group -> add_detailed_vis_infos(info_geometry_t::FULL_LINE, dirt_query.solution_traj, body_name, ss);

    //*vis_group -> add_animation(dirt_query.solution_traj, ss, dirt_query.start_state);

    //*vis_group -> output_html("output.html");

    //*delete vis_group;

    std::cout<<"End of program"<<std::endl;
}
