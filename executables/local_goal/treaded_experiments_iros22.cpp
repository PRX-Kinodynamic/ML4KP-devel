#ifndef TORCH_NOT_BUILT
#include "prx/utilities/defs.hpp"
#include "prx/utilities/learned_modules/learned_expand.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/planners/dirt.hpp"
#include "prx/planning/planner_statistics.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/visualization/three_js_group.hpp"

using namespace prx;

int main(int argc, char* argv[])
{
    try
    {
        std::string params_file;
        if (argc <= 1)
        {
            prx_throw("The planner evaluation executable needs a parameter file!");
        }
        else 
        {
            params_file = std::string(argv[1]);
        }
        
        param_loader params(params_file);
        params.print();
        simulation_step = params["simulation_step"].as<double>();
        int random_seed = params["random_seed"].as<int>();
        init_random(random_seed);

        auto obstacles = load_obstacles(params["environment"].as<std::string>());
        auto obstacle_list = obstacles.second;
        auto obstacle_names = obstacles.first;

        std::string plant_name = params["/plant/name"].as<std::string>();
        std::string plant_path = params["/plant/path"].as<std::string>();
        auto plant = system_factory_t::create_system(plant_name,plant_path);
        prx_assert(plant != nullptr, "Plant is nullptr!");

        std::vector<double> lower_bounds = params["/plant/state_space_lower_bound"].as<std::vector<double>>();
        std::vector<double> upper_bounds = params["/plant/state_space_upper_bound"].as<std::vector<double>>();
        plant -> set_state_space_bounds(lower_bounds,upper_bounds);

        world_model_t world_model({plant},{obstacle_list});
        world_model.create_context("planning_context",{plant_name},{obstacle_names});
        auto context = world_model.get_context("planning_context");

        auto ss = context.first->get_state_space();
        auto cs = context.first->get_control_space();
        auto sg = context.first;

        dirt_t dirt("dirt");
        dirt_specification_t dirt_spec(context.first,context.second);
        dirt_spec.min_control_steps = params["/plant/min_steps"].as<int>();
        dirt_spec.max_control_steps = params["/plant/max_steps"].as<int>();
        dirt_spec.blossom_number = params["blossom_number"].as<int>();
        dirt_spec.use_pruning = false;

        // Define distance function, heuristi function here

        dirt_query_t dirt_query(ss,cs);
        dirt_query.start_state = context.first->get_state_space()->make_point();
		context.first->get_state_space()->copy_point_from_vector(dirt_query.start_state,params["/plant/start_state"].as<std::vector<double>>());
		dirt_query.goal_state = context.first->get_state_space()->make_point();
		context.first->get_state_space()->copy_point_from_vector(dirt_query.goal_state,params["/plant/goal_state"].as<std::vector<double>>());

        // Define goal check function here
        dirt_query.goal_region_radius = params["goal_radius"].as<double>();
        dirt_query.goal_check = [&,ss](space_point_t point)
        {
            return ss -> euclidean_2d(point, dirt_query.goal_state, 0, 3) < dirt_query.goal_region_radius;
        };

        learned_expand_t learned_expand(dirt_spec,dirt_query, sg);
        learned_expand.init(params);
        
        dirt_spec.expand = [&learned_expand](space_point_t& s, std::vector<plan_t*>& plans, std::vector<trajectory_t*>& trajs, int bn, bool blossom_expand)
        {
            learned_expand.expand(s,plans,trajs,bn,blossom_expand);
        };

        int stats_runs = params["stats_runs"].as<int>();
        condition_check_t checker(params["checker_type"].as<std::string>(),params["checker_value"].as<double>());
        double stats_iters = params["stats_iters"].as<double>();
        std::ofstream fout;

        for( int i = 0; i < stats_runs; ++i )
        {
            dirt.link_and_setup_spec(&dirt_spec);
            dirt.preprocess();
            dirt.link_and_setup_query(&dirt_query);

            planner_statistics_t stats;
            stats.link_planner(&dirt);
            stats.link_criterion(&checker);
            stats.repeat_data_gathering(stats_iters);

            std::string full_filename = output_path+params["output_dir"].as<std::string>()+params["planner_name"].as<std::string>()+"_"+std::to_string(i)+".txt";
			fout.open(full_filename);
			fout<<stats.serialize() << std::endl;
			fout.close();

            // TODO: Add visualization code here.
            if (true)
            {
                dirt.fulfill_query();
                std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();
                three_js_group_t* vis_group = new three_js_group_t({plant},{obstacle_list});
                vis_group->add_vis_infos(info_geometry_t::LINE, dirt_query.tree_visualization, body_name, ss, "0x000000");
                vis_group->output_html(params["output_dir"].as<std::string>()+params["planner_name"].as<std::string>()+"_"+std::to_string(i)+".html");
                delete vis_group;
            }
        }
    }
    catch(const prx_assert_t& e) 
    {
        std::cerr << e.what() << std::endl;
        return -1;
    }
}
#else
int main() {}
#endif