#include "prx/utilities/defs.hpp"
#include "prx/simulation/system_factory.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planner_functions/planner_functions.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/planning/planners/prm.hpp"
#include "prx/visualization/three_js_group.hpp"

using namespace prx;

int main(int argc, char* argv[])
{
    int planner_iterations = 300;
    int stats_iterations = 30;
    
    simulation_step = 0.1;
    init_random(111093);
    //init_random(111);
    
    // auto obstacles = load_obstacles("environments/rrt_star_obstacles.yaml");
    auto obstacles = load_obstacles("environments/kmp_benchmark/unicycle_fo/bugtrap_0_ad_new.yaml");
    std::vector<std::shared_ptr<movable_object_t>> obstacle_list = obstacles.second;
    std::vector<std::string> obstacle_names = obstacles.first;

    // std::string plant_name = "FO_treaded_vehicle";
    // std::string plant_type = "FO_treaded_vehicle";
    std::string plant_name = "FO_unicycle";
    std::string plant_type = "FO_unicycle";
    auto plant = system_factory_t::create_system(plant_type,plant_name);
    prx_assert(plant != nullptr, "Plant is nullptr!");
    
    world_model_t world_model({plant},{obstacle_list});
    world_model.create_context("planning_context",{plant_name},{obstacle_names});
    auto context = world_model.get_context("planning_context");
    auto ss = context.first -> get_state_space();
    
    prm_t prm("prm");
    prm_specification_t prm_spec(context.first,context.second);
    prm_spec.k = 10;
    prm_spec.M = 2000;

    prm_query_t prm_query(ss,context.first->get_control_space());

    bool get_visualization = true;
    prm_query.get_visualization = get_visualization;

    

    //start and goal states
    	//bugtrap
	std::vector<double> start_vec = std::vector<double>({3.8, 3, 0}); 
	std::vector<double> goal_vec = std::vector<double>({5.2, 3, 0});
	
	//kink
	//std::vector<double> start_vec = std::vector<double>({0.5, 4.5, 1.55}); 
	//std::vector<double> goal_vec = std::vector<double>({5.5, 4, 1.55});
	
	//parallelpark
	//std::vector<double> start_vec = std::vector<double>({0.7, 0.8, 0}); 
	//std::vector<double> goal_vec = std::vector<double>({1.9, 0.3, 0});
	
	//wall
	//std::vector<double> start_vec = std::vector<double>({1.5, 2.5, 0}); 
	//std::vector<double> goal_vec = std::vector<double>({3.5, 2.5, 0});
	
	prm_query.start_state = context.first->get_state_space()->make_point();
	context.first->get_state_space()->copy_point_from_vector(prm_query.start_state,start_vec);
	
	prm_query.goal_state = context.first->get_state_space()->make_point();
	context.first->get_state_space()->copy_point_from_vector(prm_query.goal_state,goal_vec);
	
	
	// std::cout << "start" << prm_query.start_state << std::endl;
	
	prm.link_and_setup_spec(&prm_spec);

    	prm.preprocess();

    	prm.link_and_setup_query(&prm_query);
    	
    	condition_check_t checker("time",  10);
    	
    	prm.resolve_query(&checker);
    	
    	 
	
	
	/*prm_query.goal_region_radius = 0.1;
	

	
        prm_spec.h = [&](space_point_t a, space_point_t b)
        {
            // changes here
            
            // return prm.get_closest_cost(a) / max_velocity ;
            
            return prm_spec.distance_function(a,b) / 0.5;
        };

        prm_query.goal_check = [&,prm_spec](space_point_t s)
        {
            return prm_spec.distance_function(s,prm_query.goal_state) < prm_query.goal_region_radius; 
        };

		condition_check_t checker("time",  planner_iterations/stats_iterations);
		// condition_check_t checker("time",  10);
		std::ofstream fout;

		int stats_runs = 10 ;// params["planner_runs"].as<int>();
		for (int i = 0; i < stats_runs; i++)
		{				
			prm.link_and_setup_spec(&prm_spec);
			prm.preprocess();
			prm.link_and_setup_query(&prm_query);
			
			planner_statistics_t stats;
			stats.link_planner(&prm);
			stats.link_criterion(&checker);
			// stats.repeat_data_gathering(30);
			stats.repeat_data_gathering(stats_iterations);

			std::string full_filename = lib_path+"out/kmp_benchmark"+"prm"+"_"+std::to_string(i)+".txt";
			fout.open(full_filename);
			fout<<stats.serialize() << std::endl;
			fout.close();
			
			prm_query.clear_outputs();
			prm.reset();
    		}*/
    	
    
    // visualization not working
    /*if(get_visualization)
	{
		std::cout<< "here1" << std::endl;
		prm.fulfill_query();
		std::string vis_body = "FO_treaded_vehicle/body";
		three_js_group_t* vis_group = new three_js_group_t({plant},{obstacle_list});

		for(auto& traj : prm_query.tree_visualization)
		{
			vis_group->add_vis_infos(info_geometry_t::LINE, traj, vis_body, context.first->get_state_space());
		}
		std::cout<< "here2" << std::endl;
		if (prm_query.solution_cost > 0)
		// if (dirt_query.solution_plan.duration() > 0)
		{
			vis_group->add_vis_infos(info_geometry_t::FULL_LINE, prm_query.solution_traj, vis_body, context.first->get_state_space(),"0x00ffff");
			double timestamp=0;
			for(auto state : prm_query.solution_traj)
			{
				context.first->get_state_space()->copy_from_point(state);
				vis_group->snapshot_state(timestamp);
				timestamp+=simulation_step;
			}
		}
		else
		{
			double timestamp=0;
			context.first->get_state_space()->copy_from_point(prm_query.start_state);
			vis_group->snapshot_state(timestamp);
			timestamp+=simulation_step;
		}
		std::cout<< "here3" << std::endl;
		vis_group->output_html("out_"+std::to_string(1)+".html");
		delete vis_group;
	}
	prm_query.clear_outputs();
	prm.reset();*/
    
    
    
    
}
