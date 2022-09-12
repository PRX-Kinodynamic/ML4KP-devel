#ifndef TORCH_NOT_BUILT
#include "prx/utilities/defs.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/utilities/learned_modules/learned_controller.hpp"
#include "prx/utilities/learned_modules/reachable_region_roadmap.hpp"
#include "prx/utilities/learned_modules/ground_truth_roadmap.hpp"
#include "prx/utilities/learned_modules/access_roadmap.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/planning/planners/dirt.hpp"

#include <fstream>

using namespace prx;

int main(int argc, char* argv[])
{
    try
    {
        std::string params_file;
        if (argc <= 1)
        {
            prx_throw("This executable needs a parameter file!");
        }
        else 
        {
            params_file = std::string(argv[1]);
        }
        
        param_loader params(params_file);
        params.print();
        prx::timer_t timer; 
        simulation_step = params["simulation_step"].as<double>();
        int random_seed = params["random_seed"].as<int>();
        init_random(random_seed);

        auto obstacles = load_obstacles(params["environment"].as<std::string>());
        auto obstacle_list = obstacles.second;
        auto obstacle_names = obstacles.first;

        std::string plant_name = params["/plant/name"].as<std::string>();
        std::string plant_path = params["/plant/path"].as<std::string>();
        auto plant = system_factory_t::create_system(plant_name,plant_path);

        std::vector<double> lower_bounds = params["/plant/state_space_lower_bound"].as<std::vector<double>>();
        std::vector<double> upper_bounds = params["/plant/state_space_upper_bound"].as<std::vector<double>>();
        plant -> set_state_space_bounds(lower_bounds,upper_bounds);

        world_model_t world_model({plant},{obstacle_list});
        world_model.create_context("planning_context",{plant_name},{obstacle_names});
        auto context = world_model.get_context("planning_context");

        auto ss = context.first->get_state_space();
        auto cs = context.first->get_control_space();
        auto sg = context.first;

        dirt_specification_t dirt_spec(context.first,context.second);
        dirt_query_t dirt_query(ss,cs);
        dirt_query.start_state = ss -> make_point();
        dirt_query.goal_state  = ss -> make_point();

        dirt_query.goal_region_radius = params["goal_radius"].as<double>();

        dirt_spec.distance_function = [&](space_point_t a, space_point_t b)
        {
            std::vector <double> diff = {a->at(0)-b->at(0),a->at(1)-b->at(1),
            norm_angle_pi(a->at(2)-b->at(2))};

            double accum = 0.;
            for (auto v: diff) {
                accum += v*v;
            }
            return sqrt(accum);
        };

        learned_controller_t controller(params);

        dirt_query.goal_check = [&,dirt_spec,ss](space_point_t s)
        {
            return dirt_spec.distance_function(s,dirt_query.goal_state) < dirt_query.goal_region_radius; 
        };

        // reachable_region_roadmap_t rrr(params);
        int max_failures = params["num_failures"].as<int>();
        ground_truth_roadmap_t rrr;
        rrr.set_max_failures(max_failures);
        double roadmap_time_taken = 0.0;
        timer.reset();

        rrr.build_roadmap(dirt_query, dirt_spec, controller);

        std::cout << "Finished constructing the graph." << std::endl;

        bool is_connected = rrr.is_connected();
        // while (!is_connected)
        // {
        //     rrr.refine_roadmap(dirt_spec);
        //     is_connected = rrr.is_connected();
        // }

        roadmap_time_taken += timer.measure();
        std::cout << "Time taken for roadmap construction: " << roadmap_time_taken << std::endl;
        
        // rrr.get_validation_accuracy();
        std::vector<double> s = params["/plant/start_state"].as<std::vector<double>>();
        std::vector<double> g = params["/plant/goal_state"].as<std::vector<double>>();

        dirt_t dirt("dirt");
        dirt_spec.min_control_steps = params["/plant/min_steps"].as<int>();
        dirt_spec.max_control_steps = params["/plant/max_steps"].as<int>();
        dirt_spec.blossom_number = 5;
        dirt_spec.use_pruning = false;

        condition_check_t checker("solutions",1);

        double time_taken = 0.0;
        timer.reset();
        ss -> copy_point_from_vector(dirt_query.start_state,s);
        auto s_nn = rrr.add_start(dirt_query.start_state, dirt_spec, dirt_query, controller);
        ss -> copy_point_from_vector(dirt_query.goal_state,g);
        auto g_nn = rrr.add_goal(dirt_query.goal_state, dirt_spec, dirt_query, controller);

        prx_assert(s_nn != -1 && g_nn != -1, "Could not find a start or goal node!");
        
        auto path = rrr.get_shortest_path(s_nn,g_nn);
        for (auto v : path)
        {
            std::cout << v << " ";
        }
        std::cout << std::endl;

        /*
        trajectory_t full_traj(ss);
        space_point_t current = ss -> make_point();

        for (int i = 0; i <= path.size(); i++)
        {
            if (i == 0)
            {
                ss -> copy_point(dirt_query.goal_state,rrr.get_point(path[i]));
                std::cout << "Start: " << ss -> print_point(dirt_query.start_state,2) << std::endl;
                std::cout << "Goal: " << ss -> print_point(dirt_query.goal_state,2) << std::endl;
                controller.fulfill_query(dirt_query, dirt_spec);
                if (!dirt_spec.valid_check(dirt_query.solution_traj))
                {
                    dirt_query.clear_outputs();
                    dirt.reset();
                    checker.reset();
                    dirt.link_and_setup_spec(&dirt_spec);
                    dirt.preprocess();
                    dirt.link_and_setup_query(&dirt_query);
                    dirt.resolve_query(&checker);
                    dirt.fulfill_query();
                    if (dirt_query.solution_traj.size() ==0) break;
                }
            }
            else if (i < path.size())
            {
                ss -> copy_point(dirt_query.start_state, full_traj.back());
                ss -> copy_point(dirt_query.goal_state,rrr.get_point(path[i]));
                std::cout << "Start: " << ss -> print_point(dirt_query.start_state,2) << std::endl;
                std::cout << "Goal: " << ss -> print_point(dirt_query.goal_state,2) << std::endl;
                controller.fulfill_query(dirt_query, dirt_spec);
                if (!dirt_spec.valid_check(dirt_query.solution_traj))
                {
                    dirt_query.clear_outputs();
                    dirt.reset();
                    checker.reset();
                    dirt.link_and_setup_spec(&dirt_spec);
                    dirt.preprocess();
                    dirt.link_and_setup_query(&dirt_query);
                    dirt.resolve_query(&checker);
                    dirt.fulfill_query();
                    if (dirt_query.solution_traj.size() ==0) break;
                }
            }
            else
            {
                ss -> copy_point(dirt_query.start_state, full_traj.back());
                ss -> copy_point_from_vector(dirt_query.goal_state,g);
                std::cout << "Start: " << ss -> print_point(dirt_query.start_state,2) << std::endl;
                std::cout << "Goal: " << ss -> print_point(dirt_query.goal_state,2) << std::endl;
                controller.fulfill_query(dirt_query, dirt_spec);
                if (!dirt_spec.valid_check(dirt_query.solution_traj))
                {
                    dirt_query.clear_outputs();
                    dirt.reset();
                    checker.reset();
                    dirt.link_and_setup_spec(&dirt_spec);
                    dirt.preprocess();
                    dirt.link_and_setup_query(&dirt_query);
                    dirt.resolve_query(&checker);
                    dirt.fulfill_query();
                    if (dirt_query.solution_traj.size() ==0) break;
                }
            }

            full_traj += dirt_query.solution_traj;
            dirt_query.clear_outputs();
        }
        time_taken += timer.measure();
        std::cout << "Time: " << std::fixed << std::setprecision(16) <<
         time_taken << std::endl;
         */

        // Output graph to file.
        std::string vertex_fname = output_path + "vertices.txt";
        std::string edge_fname = output_path + "/edges.txt";

        std::ofstream vertex_file(vertex_fname);
        std::ofstream edge_file(edge_fname);

        vertex_file << rrr.print_vertices(ss) << std::endl;
        edge_file << rrr.print_edges() << std::endl;

        vertex_file.close();
        edge_file.close();
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