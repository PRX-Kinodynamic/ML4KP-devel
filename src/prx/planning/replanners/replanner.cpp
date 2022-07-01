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
            planning_time = params["planning_time"].as<double>();
            max_replanning_cycles = params["max_replanning_cycles"].as<int>();
            horizon = params["horizon"].as<double>();

            // Some asserts for sanity.
            prx_assert(planning_time > 0 && planning_time <= horizon, "Invalid parameters for replanner.");

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
            control_space = rrt_spec->control_space;
        }

        void replanner_t::resolve_query()
        {
            prx_assert(planner != NULL && rrt_spec != NULL && rrt_query != NULL,"Planner not initialized");
            prx_assert(sim != NULL,"World model not initialized");
            rrt_spec->planning_time = planning_time;
            full_solution_trajectory = new trajectory_t(state_space);
            space_point_t final_state = state_space -> make_point();
            double multiplier = 1.0/simulation_step;
            do
            {
                // Increment the cycle and update the underlying planner's horizon.
                current_cycle += 1;
                std::cout << "Cycle: " << current_cycle << std::endl;
                rrt_spec -> horizon = rrt_query -> start_time + horizon;

                // Perform the planning cycle.
                perform_single_planning_cycle();

                tree_visualization.clear();
                if (rrt_query -> get_visualization)
                    for (auto e : rrt_query -> tree_visualization)
                        tree_visualization.push_back(e);

                //  Now we have a plan.
                if (rrt_query -> solution_traj.size() == 0)
                {
                    // std::cout << state_space -> print_point(rrt_query -> start_state,4) << std::endl;
                    // Apply the fallback for the next cycle.
                    rrt_query->solution_plan.append_onto_back(planning_time);
                    rrt_spec -> stopping_control(rrt_query->start_state, planning_time);
                    control_space -> copy_to_point(rrt_query -> solution_plan.back().control);
                    control_space -> enforce_bounds(rrt_query -> solution_plan.back().control);
                    rrt_spec -> propagate(rrt_query -> start_state, rrt_query -> solution_plan, rrt_query -> solution_traj);
                }
                if (rrt_query -> solution_traj.size() < planning_time*multiplier)
                {
                    auto final_state = rrt_query -> solution_traj.back();
                    std::cout << "Found solution length: " << rrt_query -> solution_traj.size() << std::endl;
                    if(!rrt_query->goal_check(final_state))
                    {
                        rrt_query->solution_plan.append_onto_back(planning_time - rrt_query -> solution_cost);
                        rrt_spec -> stopping_control(rrt_query->start_state, planning_time);
                        control_space -> copy_to_point(rrt_query -> solution_plan.back().control);
                        control_space -> enforce_bounds(rrt_query -> solution_plan.back().control);
                        rrt_spec -> propagate(rrt_query -> start_state, rrt_query -> solution_plan, rrt_query -> solution_traj);
                        std::cout << "So this happened." << std::endl;
                    }
                }

                unsigned next_execution_index = std::min(planning_time*multiplier, (rrt_query -> solution_traj.size() - 1.0));
                auto next_execution_state = rrt_query -> solution_traj.at(next_execution_index);

                // We have to do this otherwise there may be duplicates.
                trajectory_t copy_traj(rrt_query -> solution_traj);
                state_space -> copy_point(final_state, next_execution_state);
                copy_traj.resize(next_execution_index);
                *full_solution_trajectory += copy_traj;

                // Check if the current execution cycle would lead to a collision.
                // This part is going to executed no matter what, so if there's a collision here,
                // then the planner screwed up.
                // std::cout << "Solution traj size: " << rrt_query -> solution_traj.size() << std::endl;
                // std::cout << "Next execution index: " << next_execution_index << std::endl;
                for (unsigned i = 0; i <= next_execution_index && continue_planning; i++)
                {
                    sim -> update_all_obstacle_poses(rrt_query -> start_time + i * simulation_step);
                    auto step_state = rrt_query -> solution_traj.at(i);
                    std::cout << "Checking state " << state_space->print_point(step_state,4) <<
                     " @ " << rrt_query -> start_time + i * simulation_step << std::endl;
                    if (!rrt_spec -> valid_state(step_state))
                        std::cout << "Collision during execution! t = " << rrt_query -> start_time + i * simulation_step << std::endl;
                    continue_planning &= rrt_spec -> valid_state(step_state);
                }

                continue_planning &= !rrt_query -> goal_check(next_execution_state);

                // Update the planning info for the next planning cycle.
                sim -> update_all_obstacle_poses(rrt_query -> start_time);

                // Update the start state for the next planning cycle.
                state_space -> copy_point(rrt_query -> start_state, next_execution_state);
                // TODO: Could this be less than the planning time?
                rrt_query -> start_time += planning_time;
                // std::cout << "Continue planning? " << continue_planning << std::endl;

            } while (continue_planning && current_cycle < max_replanning_cycles);
            full_solution_trajectory->copy_onto_back(final_state);
        }
}