#ifndef TORCH_NOT_BUILT
#include "prx/utilities/defs.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/utilities/learned_modules/learned_controller.hpp"
#include "prx/utilities/learned_modules/planning/strict_reachable_roadmap.hpp"
#include "prx/utilities/learned_modules/planning/reachable_roadmap.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/planning/planners/dirt.hpp"
#include "prx/planning/planner_statistics.hpp"
#include "prx/visualization/three_js_group.hpp"

#include <fstream>

#ifdef __cpp_lib_filesystem
    #include <filesystem.hpp>
    namespace fs = std::filesystem;
#else
    #define _LIBCPP_NO_EXPERIMENTAL_DEPRECATION_WARNING_FILESYSTEM
    #include <experimental/filesystem>
    namespace fs = std::experimental::filesystem;
#endif

using namespace prx;


int main(int argc, char* argv[])
{
    try
    {
        // pull param file (worth a read)
        std::string params_file= "local_goal/controller_test.yaml";
        param_loader params(params_file);
        params.print();

        prx::timer_t timer; 
        simulation_step = params["simulation_step"].as<double>();
        int random_seed = params["random_seed"].as<int>();
        init_random(random_seed);
        torch::set_num_threads(1);

        std::cout << "Params Loaded" << std::endl;


        //unimportant, as empty.yaml has no obstacles (aside from boundaries)
        auto obstacles = load_obstacles(params["environment"].as<std::string>());
        auto obstacle_list = obstacles.second;
        auto obstacle_names = obstacles.first;
        std::cout << "Obstacles Loaded" << std::endl;

        // load plant. While i am worried that the dynamics could be different, it is failing on inference first.
        std::string plant_name = params["/plant/name"].as<std::string>();
        std::string plant_path = params["/plant/path"].as<std::string>();
        auto plant = system_factory_t::create_system(plant_name,plant_path);

        std::cout << "System Loaded" << std::endl;


        // load world. v easy. no problems
        std::vector<double> lower_bounds = params["/plant/state_space_lower_bound"].as<std::vector<double>>();
        std::vector<double> upper_bounds = params["/plant/state_space_upper_bound"].as<std::vector<double>>();
        plant -> set_state_space_bounds(lower_bounds,upper_bounds);

        world_model_t world_model({plant},{obstacle_list});
        world_model.create_context("planning_context",{plant_name},{obstacle_names});
        auto context = world_model.get_context("planning_context");

        std::cout << "World Constructed" << std::endl;



        
        auto ss = context.first->get_state_space();
        auto cs = context.first->get_control_space();
        auto sg = context.first;

        dirt_specification_t dirt_spec(context.first,context.second);
        dirt_spec.sample_state = [ss](space_point_t& s)
        {
        ss -> sample(s); s -> at(3) = s -> at(4) = 0.0;
        };
        dirt_query_t dirt_query(ss,cs);
        dirt_query.start_state = ss -> make_point();
        dirt_query.goal_state  = ss -> make_point();
        dirt_query.get_visualization = true;

        dirt_query.goal_region_radius = params["goal_radius"].as<double>();

        dirt_spec.distance_function = [&](space_point_t a, space_point_t b)
        {
            return ss -> euclidean_2d(a,b,0,2);
        };

        dirt_spec.h = [&](space_point_t s, space_point_t d)
        {
            return ss -> euclidean_2d(s,d,0,2);
        };

        dirt_query.goal_check = [&,dirt_spec,ss](space_point_t s)
        {
            // return dirt_spec.distance_function(s,dirt_query.goal_state) < dirt_query.goal_region_radius; 
            return ss -> euclidean_2d(s, dirt_query.goal_state, 0, 3) < dirt_query.goal_region_radius;
        };
        
        std::cout << "DIRT spec Initialized" << std::endl;

        learned_controller_t controller(params);

        std::cout << "Controller Loaded" << std::endl;

        dirt_t dirt("dirt");
        dirt_spec.min_control_steps = params["/plant/min_steps"].as<int>();
        dirt_spec.max_control_steps = params["/plant/max_steps"].as<int>();
        dirt_spec.blossom_number = 1;
        dirt_spec.use_pruning = false;

        std::cout << "DIRT Initialized" << std::endl;

        std::vector<double> s = params["start_state"].as<std::vector<double>>();
        std::vector<double> g = params["goal_state"].as<std::vector<double>>();

        ss -> copy_point_from_vector(dirt_query.start_state,s);
        ss -> copy_point_from_vector(dirt_query.goal_state,g);


            
        std::cout<<"Start trial: "<<std::endl; 

        std::vector<double> pt_vec;
        space_point_t pt;

        std::cout << (ss->print_point(dirt_query.start_state)) << std::endl;
        
        for( double val: pt_vec){
            std::cout << val << ", ";
        }
        std::cout << std::endl;



        dirt_query.clear_outputs();

        controller.fulfill_query(dirt_query, dirt_spec);
        
        if (dirt_spec.valid_check(dirt_query.solution_traj) && dirt_query.solution_traj.size() > 1){
            std::cout << "Did not reach goal."<< std::endl;
        }

        std::cout << "trajectory"<< std::endl;

        for (auto st : dirt_query.solution_traj){
            std::cout << (ss->print_point(st)) << std::endl;
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