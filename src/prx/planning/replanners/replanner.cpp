#include "prx/planning/replanners/replanner.hpp"

namespace prx
{

        replanner_t::replanner_t(const std::string& name) 
        {
            planner_name = name;
            reset();
        }

        replanner_t::~replanner_t()
        {
        }

        void replanner_t::setup(param_loader params)
        {
            buffer_time = params["buffer_time"].as<double>();
            planning_time = params["planning_time"].as<double>();
            max_replanning_cycles = params["max_replanning_cycles"].as<int>();
            horizon = params["horizon"].as<double>();

            // Some asserts for sanity.
            prx_assert(buffer_time > 0 && planning_time > 0 && buffer_time + planning_time <= horizon, "Invalid parameters for replanner.");

            checker = new condition_check_t("time",planning_time);
        }

        void replanner_t::link_world_model(std::shared_ptr<world_model_t> wm)
        {
            sim = wm;
        }

        void replanner_t::link_planner(rrt_t* _planner, rrt_specification_t* spec, rrt_query_t* query)
        {
            planner = _planner;
            rrt_spec = spec;
            rrt_query = query;

            state_space = rrt_spec->state_space;
        }

        void replanner_t::resolve_query()
        {
            prx_assert(planner != NULL && rrt_spec != NULL && rrt_query != NULL,"Planner not initialized");
            prx_assert(sim != NULL,"World model not initialized");
            full_solution_trajectory = new trajectory_t(state_space);
            space_point_t final_state = state_space -> make_point();
            do
            {
                // Increment the cycle and update the underlying planner's horizon.
                current_cycle += 1;
                std::cout << "Cycle: " << current_cycle << std::endl;
                std::cout << "Current time: " << rrt_query -> start_time << std::endl;
                std::cout << "Planning from: " << state_space -> print_point(rrt_query -> start_state,4) << std::endl;
                // std::cout << "State valid? " << rrt_spec -> valid_state(rrt_query -> start_state) << std::endl; 
                // rrt_spec -> horizon = (current_cycle + 1) * horizon;
                rrt_spec -> horizon = rrt_query -> start_time + horizon;
                // std::cout << "Planning for a horizon of " << rrt_spec -> horizon << std::endl;

                // Perform the planning cycle.
                perform_single_planning_cycle();

                if (rrt_query -> get_visualization)
                {
                    for (auto e : rrt_query -> tree_visualization)
                    {
                        tree_visualization.push_back(e);
                    }
                }

                //  Now we have a plan.
                if (rrt_query -> solution_traj.size() == 0) break;
                // std::cout << "Solution cost so far: " << rrt_query -> solution_cost << std::endl;
                unsigned next_execution_index = std::min(1 + (buffer_time + planning_time)/simulation_step, rrt_query -> solution_traj.size() - 1.0);
                auto next_execution_state = rrt_query -> solution_traj.at(next_execution_index);

                // We have to do this otherwise there may be duplicates.
                trajectory_t copy_traj(rrt_query -> solution_traj);
                state_space -> copy_point(final_state, next_execution_state);
                copy_traj.resize(next_execution_index - 1);
                *full_solution_trajectory += copy_traj;

                auto first_state = rrt_query -> solution_traj.front();
                // std::cout << "First state: " << state_space -> print_point(first_state,4) << std::endl;
                // std::cout << "Traj len: " << full_solution_trajectory -> size() << std::endl;
                // std::cout << "Next execution state: " << state_space -> print_point(next_execution_state,4) << std::endl;
               
                // Check if the current execution cycle would lead to a collision.
                for (unsigned i = 0; i < next_execution_index; i++)
                {
                    sim -> update_all_obstacle_poses(rrt_query -> start_time + i * simulation_step);
                    auto step_state = rrt_query -> solution_traj.at(i);
                    continue_planning &= rrt_spec -> valid_state(step_state);
                }

                continue_planning &= !rrt_query -> goal_check(next_execution_state);

                // Update the planning info for the next planning cycle.
                // sim -> update_all_obstacle_poses(rrt_query -> start_time + buffer_time + planning_time);
                sim -> update_all_obstacle_poses(rrt_query -> start_time + buffer_time);

                // Update the start state for the next planning cycle.
                state_space -> copy_point(rrt_query -> start_state, next_execution_state);
                rrt_query -> start_time += buffer_time + planning_time;
                std::cout << "Continue planning? " << continue_planning << std::endl;

            } while (continue_planning && current_cycle < max_replanning_cycles);
            full_solution_trajectory->copy_onto_back(final_state);
        }
}