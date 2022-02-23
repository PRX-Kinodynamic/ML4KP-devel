#ifndef TORCH_NOT_BUILT
#include "prx/utilities/defs.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/utilities/learned_modules/reachability_estimator.hpp"
#include "prx/utilities/learned_modules/learned_controller.hpp"
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

        std::vector<double> lower_bounds = params["/plant/state_space_lower_bound"].as<std::vector<double>>();
        std::vector<double> upper_bounds = params["/plant/state_space_upper_bound"].as<std::vector<double>>();
        plant -> set_state_space_bounds(lower_bounds,upper_bounds);

        world_model_t world_model({plant},{obstacle_list});
        world_model.create_context("planning_context",{plant_name},{obstacle_names});
        auto context = world_model.get_context("planning_context");

        auto ss = context.first->get_state_space();
        auto cs = context.first->get_control_space();
        auto sg = context.first;

        const int num_trajectories = params["num_trajectories"].as<int>();

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
        dirt_query.goal_check = [&,ss](space_point_t s)
        {
            std::vector <double> diff = {dirt_query.goal_state->at(0)-s->at(0),
            dirt_query.goal_state->at(1)-s->at(1),
            norm_angle_pi(dirt_query.goal_state->at(2)-s->at(2))};

            double accum = 0.;
            for (auto v: diff) {
                accum += v*v;
            }
            return sqrt(accum) < dirt_query.goal_region_radius; 
            // return ss -> euclidean_2d(point, dirt_query.goal_state, 0, 3) < dirt_query.goal_region_radius;
        };

        std::string output_file = params["output_file"].as<std::string>();

        reachability_estimator_t estimator(params);
        learned_controller_t controller(params);
        double horizon = params["max_steps"].as<double>();

        std::ofstream ofs;
        ofs.open(output_path + output_file, std::ofstream::out);

        std::vector<double> start_vec, goal_vec;

        for (int i = 0; i < num_trajectories; i++)
        {
            dirt_query.clear_outputs();

            bool sample = true;
            while(sample)
            {
                ss -> sample(dirt_query.start_state);
                ss -> sample(dirt_query.goal_state);
                sample = !dirt_spec.valid_state(dirt_query.start_state) || !dirt_spec.valid_state(dirt_query.goal_state);
            }

            ss -> copy_vector_from_point(start_vec, dirt_query.start_state);
            ss -> copy_vector_from_point(goal_vec, dirt_query.goal_state);

            controller.fulfill_query(dirt_query, sg, horizon);

            if (dirt_query.solution_traj.size() == 0) continue;

            if (dirt_spec.valid_check(dirt_query.solution_traj))
            {
                // Step through the trajectory and save each intermediate state as a data point.
                unsigned last_added = 0;
                for (int i = 0; i < dirt_query.solution_traj.size(); i++)
                {
                    
                }
                // ofs << ss -> print_point(dirt_query.start_state,4) << "," <<
                //     ss -> print_point(dirt_query.goal_state,4) << "," <<
                //     ss -> print_point(dirt_query.goal_state,4) << std::endl;
            }
            else
            {
                trajectory_t lc_trajectory(dirt_query.solution_traj);
                dirt_query.clear_outputs();
                
                std::cout << ss->print_point(dirt_query.start_state,4) << "," << ss->print_point(dirt_query.goal_state,4) << std::endl;
                // Get a trajectory from the planner.
                dirt.link_and_setup_spec(&dirt_spec);
                dirt.preprocess();
                dirt.link_and_setup_query(&dirt_query);

                condition_check_t checker("time",5);

                dirt.resolve_query(&checker);
                dirt.fulfill_query();

                if (dirt_query.solution_traj.size() > 0)
                {
                    std::cout << lc_trajectory.size() << " " << dirt_query.solution_traj.size() << std::endl;
                }

                dirt.reset();
                
            }
            output_progress_bar(i*1.0/num_trajectories);
        }
    }
    catch(const prx_assert_t& e) 
    {
        std::cerr << e.what() << std::endl;
        return -1;
    }
}

#else
int main()
#endif