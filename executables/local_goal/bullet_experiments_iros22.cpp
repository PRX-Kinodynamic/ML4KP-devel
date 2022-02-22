#if !defined(TORCH_NOT_BUILT) && !defined(BULL_NOT_BUILT)
#include "prx/utilities/defs.hpp"
#include "prx/utilities/learned_modules/learned_expand.hpp"
#include "prx/planning/planners/dirt.hpp"
#include "prx/planning/planner_statistics.hpp"

#include "prx/bullet_sim/bullet_simulator.hpp"
#include "prx/bullet_sim/plants/plants.hpp"
#include "prx/bullet_sim/loaders/obstacle_loader.hpp"

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

        std::string plant_name = params["/plant/name"].as<std::string>();
        std::string plant_path = params["/plant/path"].as<std::string>();
        auto plant = system_factory_t::create_system(plant_name,plant_path);
        auto bplant = std::dynamic_pointer_cast<bullet_plant_t>(plant);

        auto bsim = std::make_shared<bullet_simulator_t>();
		bsim -> add_urdf(bullet_path + "/data/plane.urdf");
		bsim -> add_group({plant});
        std::string obstacles_file = params["environment"].as<std::string>();
        auto retval = load_obstacles(obstacles_file,bsim);

        space_point_t init_state = plant -> state_space ->make_point();
		plant->state_space->copy_point_from_vector(init_state,params["/plant/start_state"].as<std::vector<double>>());
        plant->state_space->copy_from_point(init_state);

		bsim -> initialize_simulation();
        int start_state_id = bplant -> get_state_id();

        auto context = bsim -> get_context("bullet_context");

        auto ss = context.first->get_state_space();
    	auto cs = context.first -> get_control_space();
		auto sg = context.first;
        auto cg = context.second;
        unsigned dim = ss -> get_dimension();

        bullet_collision_group_t bcg(bsim);
		auto bcg_ptr = std::make_shared<bullet_collision_group_t>(bcg);
		bsim->set_collision_group(bcg_ptr);

        dirt_t dirt("dirt");
        dirt_specification_t dirt_spec(context.first,context.second);
        dirt_spec.min_control_steps = params["/plant/min_steps"].as<int>();
        dirt_spec.max_control_steps = params["/plant/max_steps"].as<int>();
        dirt_spec.blossom_number = params["blossom_number"].as<int>();
        dirt_spec.use_pruning = false;

        PRX_DEBUG_PRINT

        // Define distance function, heuristi function here

        dirt_query_t dirt_query(ss,cs);
        dirt_query.start_state = ss->make_point();
        ss->copy_to_point(dirt_query.start_state);
        dirt_query.goal_state = ss->make_point();
        ss->copy_point_from_vector(dirt_query.goal_state,params["/plant/goal_state"].as<std::vector<double>>());

        PRX_DEBUG_PRINT
        
        // Define goal check function here
        dirt_query.goal_region_radius = params["goal_radius"].as<double>();

        // learned_expand_t learned_expand(dirt_spec,dirt_query);
        // learned_expand.init(params);
        
        // Set up learned expand here.

        PRX_DEBUG_PRINT

        int stats_runs = params["stats_runs"].as<int>();
        condition_check_t checker(params["checker_type"].as<std::string>(),params["checker_value"].as<double>());
        double stats_iters = params["stats_iters"].as<double>();
        std::ofstream fout;

        for( int i = 0; i < stats_runs; ++i )
        {
            // Bullet-specific stuff
            bsim->restoreStateFromMemory(start_state_id);            

            dirt.link_and_setup_spec(&dirt_spec);
            dirt.preprocess();
            dirt.link_and_setup_query(&dirt_query);

            planner_statistics_t stats;
            stats.link_planner(&dirt);
            stats.link_criterion(&checker);
            stats.repeat_data_gathering(stats_iters);

            dirt.fulfill_query();
            bplant -> purge_saved_states();

            std::cout << dirt_query.solution_traj.print(2) << std::endl;

            std::string full_filename = output_path+params["output_dir"].as<std::string>()+params["planner_name"].as<std::string>()+"_"+std::to_string(i)+".txt";
			fout.open(full_filename);
			fout<<stats.serialize() << std::endl;
			fout.close();

            output_progress_bar(1.0*i/stats_runs);
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