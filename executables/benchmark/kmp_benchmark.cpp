#include "prx/utilities/defs.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planner_statistics.hpp"
#include "prx/planning/planners/dirt.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/simulation/system_factory.hpp"
#include "prx/simulation/plants/plants.hpp"


#include <fstream>

using namespace prx;

int main(int argc, char* argv[])
{
    try
	{
		std::string params_file;
		if (argc<=1)
		{
			prx_throw("The planner evaluation executable needs a parameter file!");
		}
		else
		{
			params_file = std::string(argv[1]);
		}


		param_loader params(params_file);

		simulation_step = params["simulation_step"].as<double>();

		//Planner parameters
		double planner_iterations = params["planner_iterations"].as<int>();
		double stats_iterations = params["statistics_iterations"].as<int>();
		int random_seed = params["random_seed"].as<int>();
		init_random(random_seed);
		// params.print();

		//which plant we are planning for
		std::string plant_name = params["/plant/name"].as<std::string>();
		std::string plant_path = params["/plant/path"].as<std::string>();
		std::string obstacles_file = params["obstacles_file"].as<std::string>();
		auto obstacles = load_obstacles(obstacles_file);
		auto obstacle_list = obstacles.second;
		auto obstacle_names = obstacles.first;
		auto plant = system_factory_t::create_system(plant_path,plant_name);
		prx_assert(plant != nullptr, "Plant is nullptr!");

		world_model_t world_model({plant},{obstacle_list});
		world_model.create_context("planning_context",{plant_name},{obstacle_names});
		auto context = world_model.get_context("planning_context");


        dirt_t dirt("dirt");
		dirt_specification_t dirt_spec(context.first,context.second);
		dirt_spec.use_pruning = params["use_pruning"].as<bool>();
		auto ss = context.first->get_state_space();
		auto cs = context.first->get_control_space();
		ss->set_bounds(params["/plant/state_space_lower_bound"].as<std::vector<double>>(),params["/plant/state_space_upper_bound"].as<std::vector<double>>());
		cs->set_bounds(params["/plant/control_space_lower_bound"].as<std::vector<double>>(),params["/plant/control_space_upper_bound"].as<std::vector<double>>());

		dirt_query_t dirt_query(ss,cs);
		bool get_visualization = params["visualize"].as<bool>();

		//start and goal states
		std::vector<double> start_vec = params["/plant/start_state"].as<std::vector<double>>();
		std::vector<double> goal_vec = params["/plant/goal_state"].as<std::vector<double>>();
		dirt_query.start_state = context.first->get_state_space()->make_point();
		context.first->get_state_space()->copy_point_from_vector(dirt_query.start_state,start_vec);
		dirt_query.goal_state = context.first->get_state_space()->make_point();
		context.first->get_state_space()->copy_point_from_vector(dirt_query.goal_state,goal_vec);
		dirt_query.goal_region_radius = params["/plant/goal_radius"].as<double>();
		dirt_query.get_visualization = get_visualization;

		/*dirt_spec.distance_function = [&](space_point_t a, space_point_t b)
        {
            std::vector <double> diff = {a->at(0)-b->at(0),a->at(1)-b->at(1),
            norm_angle_pi(a->at(2)-b->at(2))};

			if (plant_name == "SO_unicycle")
			{
				diff.push_back(a->at(3)-b->at(3));
				diff.push_back(a->at(4)-b->at(4));
			}

            double accum = 0.;
            for (auto v: diff) {
                accum += v*v;
            }
            return sqrt(accum);
        };*/
        
        //dirt_spec.distance_function = std::bind(&space_t::euclidean_2d, _3, 0, _4, ss -> get_dimension());

	dirt_spec.distance_function = [&](space_point_t a, space_point_t b)
	{
	  return space_t::euclidean_2d(a, b, 0, a -> get_dim());
	};
	
	
        dirt_spec.h = [&,dirt_spec](space_point_t a, space_point_t b)
        {
            return dirt_spec.distance_function(a,b) / 0.5;
        };

        dirt_query.goal_check = [&,dirt_spec](space_point_t s)
        {
            return dirt_spec.distance_function(s,dirt_query.goal_state) < dirt_query.goal_region_radius; 
        };
        
		condition_check_t checker(params["condition_check"].as<std::string>(),  planner_iterations/stats_iterations);
		std::ofstream fout;

		int stats_runs = params["planner_runs"].as<int>();
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
			stats.repeat_data_gathering(stats_iterations);

			std::string full_filename = lib_path+params["data_output_folder"].as<std::string>()+params["planner_name"].as<std::string>()+"_"+std::to_string(i)+".txt";
			std::cout << full_filename << std::endl;
			fout.open(full_filename);
			fout<<stats.serialize() << std::endl;
			fout.close();

			if(get_visualization)
			{
				dirt.fulfill_query();
				std::string vis_body = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();
				three_js_group_t* vis_group = new three_js_group_t({plant},{obstacle_list});

				for(auto& traj : dirt_query.tree_visualization)
				{
					vis_group->add_vis_infos(info_geometry_t::LINE, traj, vis_body, context.first->get_state_space());
				}

				if (dirt_query.solution_cost > 0)
				// if (dirt_query.solution_plan.duration() > 0)
				{
					vis_group->add_vis_infos(info_geometry_t::FULL_LINE, dirt_query.solution_traj, vis_body, context.first->get_state_space(),"0x00ffff");
					double timestamp=0;
					for(auto state : dirt_query.solution_traj)
					{
						context.first->get_state_space()->copy_from_point(state);
						vis_group->snapshot_state(timestamp);
						timestamp+=simulation_step;
					}
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
	}
	catch(const prx_assert_t& e)
	{
		std::cout << e.get_message() << '\n';
	}
}
