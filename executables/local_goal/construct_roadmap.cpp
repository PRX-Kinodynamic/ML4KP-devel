#ifndef TORCH_NOT_BUILT
#include "prx/utilities/defs.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/utilities/learned_modules/learned_controller.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/planning/planners/dirt.hpp"
#include "prx/utilities/heuristics/medial_axis.hpp"

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

        const int num_samples = params["num_trajectories"].as<int>();

        dirt_t dirt("dirt");

        dirt_specification_t dirt_spec(context.first,context.second);
        dirt_spec.min_control_steps = params["/learned_controller/control_duration"].as<double>()/simulation_step;
        dirt_spec.max_control_steps = params["/learned_controller/control_duration"].as<double>()/simulation_step;
        dirt_spec.blossom_number = 5;
        dirt_spec.use_pruning = false;


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
        double horizon = params["max_steps"].as<double>();

        dirt_query.goal_check = [&,dirt_spec,ss](space_point_t s)
        {
            return dirt_spec.distance_function(s,dirt_query.goal_state) < dirt_query.goal_region_radius; 
            // return ss -> euclidean_2d(point, dirt_query.goal_state, 0, 3) < dirt_query.goal_region_radius;
        };

        std::vector<double> linspace_points = linspace(-14.0,14.0,21);
        std::vector<space_point_t> test_points;
        graph_nearest_neighbors_t* test_points_metric = new graph_nearest_neighbors_t(dirt_spec.distance_function);
        std::string output_dir = output_path + params["output_dir"].as<std::string>();

        for (int i = 0; i < linspace_points.size(); i++)
        {
            for (int j = 0; j < linspace_points.size(); j++)
            {
                space_point_t point = ss -> make_point();
                point->at(0) = linspace_points[i];
                point->at(1) = linspace_points[j];
                point->at(2) = uniform_random(0,2*PRX_PI);
                if (dirt_spec.valid_state(point))
                {
                    test_points.push_back(point);
                    rrt_node_t* node = new rrt_node_t();
                    node -> point = ss -> clone_point(point);
                    test_points_metric -> add_node(node);
                } 
            }
        }

        std::ofstream ofs;
        ofs.open(output_dir + "test_points.txt");
        for (auto s : test_points) ofs << ss -> print_point(s) << std::endl;
        ofs.close();

        std::vector<space_point_t> roadmap_points;
        graph_nearest_neighbors_t* roadmap_points_metric = new graph_nearest_neighbors_t(dirt_spec.distance_function);

        const int M = params["roadmap_points"].as<int>();
        for (int i = 0; i < M; i++)
        {
            space_point_t point = ss -> make_point();
            do
            {
                dirt_spec.sample_state(point);
            } while (!dirt_spec.valid_state(point));
            roadmap_points.push_back(point);
            rrt_node_t* node = new rrt_node_t();
            node -> point = ss -> clone_point(point);
            roadmap_points_metric -> add_node(node);
        }

        std::vector<space_point_t> access_fails, depart_fails;

        // Accessibility check:
        for (int i = 0; i < test_points.size(); i++)
        {
            space_point_t point = test_points[i];
            auto closest_node = static_cast<rrt_node_t*>(roadmap_points_metric->single_query(point));
            dirt_query.clear_outputs();
            ss -> copy_point(dirt_query.start_state,point);
            ss -> copy_point(dirt_query.goal_state,closest_node->point);
            controller.fulfill_query(dirt_query,sg,horizon);

            if (dirt_spec.valid_check(dirt_query.solution_traj))
            {
                // ofs.open(output_dir + "access_success_" + std::to_string(i) + ".txt");
                // ofs << dirt_query.solution_traj.print();
                // ofs.close();
            }
            else
            {
                ofs.open(output_dir + "access_failure_" + std::to_string(i) + ".txt");
                ofs << dirt_query.solution_traj.print();
                ofs.close();
                access_fails.push_back(point);
            }
            output_progress_bar(1.0*i/test_points.size());
        }

        // Departability check.

        for (int i = 0; i < test_points.size(); i++)
        {
            space_point_t point = test_points[i];
            auto closest_node = static_cast<rrt_node_t*>(roadmap_points_metric->single_query(point));
            dirt_query.clear_outputs();
            ss -> copy_point(dirt_query.goal_state,point);
            ss -> copy_point(dirt_query.start_state,closest_node->point);
            controller.fulfill_query(dirt_query,sg,horizon);

            if (dirt_spec.valid_check(dirt_query.solution_traj))
            {
                // ofs.open(output_dir + "depart_success_" + std::to_string(i) + ".txt");
                // ofs << dirt_query.solution_traj.print();
                // ofs.close();
            }
            else
            {
                ofs.open(output_dir + "depart_failure_" + std::to_string(i) + ".txt");
                ofs << dirt_query.solution_traj.print();
                ofs.close();
                depart_fails.push_back(point);
            }
            output_progress_bar(1.0*i/test_points.size());
        }

        std::cout << "Access fails: " << 1.0*access_fails.size()/test_points.size() << std::endl;
        std::cout << "Depart fails: " << 1.0*depart_fails.size()/test_points.size() << std::endl;
    
        const int max_tries = params["max_tries"].as<int>();
        trajectory_t full_traj (ss);

        int num_final_access_fails = 0;
        int num_final_depart_fails = 0;

        for (int j = 0; j < access_fails.size(); j++)
        {
            auto point = access_fails[j];
            space_point_t rand  = ss -> clone_point(point);
            auto closest_node = static_cast<rrt_node_t*>(roadmap_points_metric->single_query(point));

            for (int i = 0; i < max_tries; i++)
            {
                bool sample_again = false;
                while(sample_again)
                {
                    rand -> at(0) = point -> at(0) + uniform_random(-2.0,2.0);
                    rand -> at(1) = point -> at(1) + uniform_random(-2.0,2.0);
                    rand -> at(2) = uniform_random(0,2*PRX_PI);
                    sample_again = !dirt_spec.valid_state(rand);
                }

                dirt_query.clear_outputs();
                full_traj.clear();

                ss -> copy_point(dirt_query.start_state,point);
                ss -> copy_point(dirt_query.goal_state,rand);
                controller.fulfill_query(dirt_query,sg,horizon);

                full_traj += dirt_query.solution_traj;
                if(full_traj.size() == 0) continue;
                dirt_query.clear_outputs();
                ss -> copy_point(dirt_query.start_state,full_traj.back());
                ss -> copy_point(dirt_query.goal_state, closest_node -> point);
                controller.fulfill_query(dirt_query,sg,horizon);

                full_traj += dirt_query.solution_traj;
                if (dirt_spec.valid_check(full_traj))
                {
                    PRX_DEBUG_PRINT
                    num_final_access_fails++;
                    break;
                }
            }

            output_progress_bar(1.0*j/access_fails.size());
        }

        for (int j = 0; j < depart_fails.size(); j++)
        {
            auto point = depart_fails[j];
            space_point_t rand  = ss -> clone_point(point);
            auto closest_node = static_cast<rrt_node_t*>(roadmap_points_metric->single_query(point));

            for (int i = 0; i < max_tries; i++)
            {
                bool sample_again = false;
                while(sample_again)
                {
                    rand -> at(0) = point -> at(0) + uniform_random(-2.0,2.0);
                    rand -> at(1) = point -> at(1) + uniform_random(-2.0,2.0);
                    rand -> at(2) = uniform_random(0,2*PRX_PI);
                    sample_again = !dirt_spec.valid_state(rand);
                }

                dirt_query.clear_outputs();
                full_traj.clear();

                ss -> copy_point(dirt_query.start_state,closest_node -> point);
                ss -> copy_point(dirt_query.goal_state,rand);
                controller.fulfill_query(dirt_query,sg,horizon);

                full_traj += dirt_query.solution_traj;
                if(full_traj.size() == 0) continue;
                dirt_query.clear_outputs();
                ss -> copy_point(dirt_query.start_state,full_traj.back());
                ss -> copy_point(dirt_query.goal_state, point);
                controller.fulfill_query(dirt_query,sg,horizon);

                full_traj += dirt_query.solution_traj;
                if (dirt_spec.valid_check(full_traj))
                {
                    PRX_DEBUG_PRINT
                    num_final_depart_fails++;
                    break;
                }
            }

            output_progress_bar(1.0*j/depart_fails.size());
        }

        std::cout << "Access fails: " << 1 - 1.0*num_final_access_fails/test_points.size() << std::endl;
        std::cout << "Depart fails: " << 1 - 1.0*num_final_depart_fails/test_points.size() << std::endl;
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