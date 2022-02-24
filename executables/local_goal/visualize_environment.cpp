#include "prx/utilities/defs.hpp" 
#include "prx/simulation/system_factory.hpp"    
#include "prx/planning/world_model.hpp"     
#include "prx/planning/planners/dirt.hpp"   
#include "prx/simulation/loaders/obstacle_loader.hpp"   
#include "prx/visualization/three_js_group.hpp"     
#include "prx/planning/planner_statistics.hpp"      
#include "prx/simulation/plants/treaded_vehicle_first_order.hpp"
#include "prx/simulation/plants/treaded_vehicle.hpp"

#include <torch/torch.h>
#include <torch/script.h>
#include <bits/stdc++.h>
#include <fstream>

#include <boost/program_options.hpp>
namespace po = boost::program_options;
using namespace boost;
using namespace prx;

const double map_resolution = 0.05;

int main(int argc, char* argv[])
{
    try
    {
        simulation_step = 0.1; 
        auto obstacles = load_obstacles("/environments/warehouse.yaml");
        
        // auto obstacles = load_obstacles("obstacles/empty.yaml");
        auto obstacle_list = obstacles.second;
        auto obstacle_names = obstacles.first;

        for (auto ob: obstacle_names){
            std::cout << ob << std::endl;
        }
        // Does the system factory loader not work? - Aravind.
        std::string plant_name = "treaded_vehicle";
        // system_ptr_t plant = system_factory_t::create_system(plant_name, plant_name);
        system_ptr_t plant = create_system<treaded_vehicle_t>(plant_name);

        std::cout << plant << std::endl;

        world_model_t world_model({plant},{obstacle_list});
        world_model.create_context("disk_context",{plant_name},{obstacle_names});

        std::vector<double> lower = {0, 0,-PRX_PI, -0.7, -0.7};
        std::vector<double> upper = { 38.0, 18.0,PRX_PI, 0.7, 0.7}; 
        plant->set_state_space_bounds(lower,upper);
        const double max_vel = 0.5;

        auto context = world_model.get_context("disk_context");

        dirt_t dirt("dirt");
        dirt_specification_t dirt_spec(context.first,context.second);   //plant, obstacles

        int min_steps = 5;
        int max_steps = 50;
        dirt_spec.min_control_steps = min_steps;
        dirt_spec.max_control_steps = max_steps;

        dirt_spec.blossom_number = 5;
        dirt_spec.use_pruning = false;

        dirt_query_t dirt_query(context.first->get_state_space(), context.first->get_control_space());
        dirt_query.start_state = context.first->get_state_space()->make_point();
        dirt_query.goal_state  = context.first->get_state_space()->make_point();

        dirt_query.start_state -> at(0) = 1;
        dirt_query.start_state -> at(1) = 1;

        // std::string goal_location = vm["goal"].as<std::string>();
        dirt_query.goal_state -> at(0) = 1;
        dirt_query.goal_state -> at(1) = 2;

        dirt_query.goal_region_radius = 0.25;    //epsilon for goal check
        dirt_query.get_visualization = true;    //for visualization


        dirt.link_and_setup_spec(&dirt_spec);
        dirt.preprocess();
        dirt.link_and_setup_query(&dirt_query);

        condition_check_t checker("iterations", 1); 

        std::ofstream fout;
        int stats_runs = 1;        

        for (int i = 0; i < stats_runs; i++)
		{				
			dirt.link_and_setup_spec(&dirt_spec);
            dirt.preprocess();
            dirt.link_and_setup_query(&dirt_query);
			planner_statistics_t stats;
			stats.link_planner(&dirt);
			stats.link_criterion(&checker);
			stats.repeat_data_gathering(50);
            dirt.fulfill_query();

            three_js_group_t* vis_group = new three_js_group_t({plant},{obstacle_list});
        
            for(auto& traj : dirt_query.tree_visualization)
            {
                if(traj != dirt_query.solution_traj)
                vis_group->add_vis_infos(info_geometry_t::FULL_LINE, traj, plant_name+"/body", context.first->get_state_space());
            }
            if(dirt_query.solution_traj.size()!=0)
            {
                vis_group->add_vis_infos(info_geometry_t::FULL_LINE, dirt_query.solution_traj,
                    plant_name+"/body", context.first->get_state_space(), "0xFF0000");
            }

            double timestamp=0;
            for(auto state : dirt_query.solution_traj)
            {
                context.first->get_state_space() -> copy_from_point(state);
                vis_group->snapshot_state(timestamp);
                timestamp+=simulation_step;
            }
            if(dirt_query.solution_traj.size()==0)
            {
                context.first->get_state_space() -> copy_from_point(dirt_query.start_state);
                vis_group->snapshot_state(timestamp);
                timestamp+=simulation_step;
            }

            std::string out_filename = "/output_"+std::to_string(i+2)+".html";
            vis_group->output_html(out_filename);
            delete vis_group;

            dirt_query.clear_outputs();
            dirt.reset();
        }
    }
    catch(const prx_assert_t& e)
    {
        std::cout << e.get_message() << '\n';
    }
    std::cout << "End of program" << std::endl;
}