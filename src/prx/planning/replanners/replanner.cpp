#include "prx/planning/replanners/replanner.hpp"

namespace prx
{

        replanner_t::replanner_t(const std::string& name, dirt_replan_specification_t* spec) : full_solution_trajectory(spec->state_space)
        {
            planner_name = name;
            reset();

            waypoint_function = [&](const space_point_t& s, const std::vector<double>& o_infos)
            {
                return std::vector<double>();
            };
        }

        replanner_t::~replanner_t()
        {
            delete checker;
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

        void replanner_t::link_planner(dirt_replan_t* _planner, dirt_replan_specification_t* spec, dirt_replan_query_t* query)
        {
            planner = _planner;
            rrt_spec = spec;
            rrt_query = query;

            state_space = rrt_spec->state_space;
            control_space = rrt_spec->control_space;

            global_goal_state = state_space -> clone_point(rrt_query->goal_state);
        }

        void replanner_t::resolve_query()
        {
            prx_assert(planner != NULL && rrt_spec != NULL && rrt_query != NULL,"Planner not initialized");
            prx_assert(sim != NULL,"World model not initialized");
            full_solution_trajectory.clear();
            rrt_spec->planning_time = planning_time;
            space_point_t final_state = state_space -> make_point();
            space_point_t step_state  = state_space -> make_point();
            space_point_t next_execution_state = state_space -> make_point();
            double multiplier = 1.0/simulation_step;
            double safety_time;
            predicted_waypoints.clear();

            std::vector<double> obstacle_infos;

            do
            {
                safety_time = simulation_step;
                // Increment the cycle and update the underlying planner's horizon.
                current_cycle += 1;
                std::cout << "Cycle: " << current_cycle << " t: " << rrt_query->start_time << std::endl;
                rrt_spec -> horizon = rrt_query -> start_time + horizon;

                obstacle_infos.clear();

                for (int i = 0; i < 5; i++)
                {
                    auto res = sim -> get_obstacle_pose("box_"+std::to_string(i),rrt_query->start_time);
                    std::cout << res.at(0) << " " << res.at(1) << " " << res.at(3) << " " << res.at(4) << std::endl;
                    obstacle_infos.push_back(res.at(0));
                    obstacle_infos.push_back(res.at(1));
                    obstacle_infos.push_back(res.at(3));
                    obstacle_infos.push_back(res.at(4));
                }

                std::vector<double> sorted_obstacle_idxes, sorted_obstacle_infos, obstacle_distances;
                for (int i = 0; i < 5; i++)
                {
                    sorted_obstacle_idxes.push_back(i);
                    obstacle_distances.push_back(
                        pow(obstacle_infos.at(i*4) - rrt_query->start_state->at(0),2) +
                        pow(obstacle_infos.at(i*4+1) - rrt_query->start_state->at(1),2)
                    );
                }

                std::sort(sorted_obstacle_idxes.begin(),sorted_obstacle_idxes.end(),
                    [&](int a, int b)
                    {
                        return obstacle_distances.at(a) < obstacle_distances.at(b);
                    }
                );

                for (int i = 0; i < 5; i++)
                {
                    sorted_obstacle_infos.push_back(obstacle_infos.at(sorted_obstacle_idxes.at(i)*4));
                    sorted_obstacle_infos.push_back(obstacle_infos.at(sorted_obstacle_idxes.at(i)*4+1));
                    sorted_obstacle_infos.push_back(obstacle_infos.at(sorted_obstacle_idxes.at(i)*4+2));
                    sorted_obstacle_infos.push_back(obstacle_infos.at(sorted_obstacle_idxes.at(i)*4+3));
                }


                // auto waypt = waypoint_function(rrt_query->start_state,obstacle_infos);
                auto waypt = waypoint_function(rrt_query->start_state,sorted_obstacle_infos);
                predicted_waypoints.push_back(waypt);
                state_space -> copy_point_from_vector(rrt_query->goal_state,waypt);
                rrt_query->goal_region_radius = 0.1;
                std::cout << "Planning for waypoint: " << state_space -> print_point(rrt_query->goal_state) << std::endl;
                // Perform the planning cycle.
                perform_single_planning_cycle();

                // node_index_t best_index = planner->get_best_node_index();
                // planner->tree_retain(best_index);

                tree_visualization.clear();
                if (rrt_query -> get_visualization)
                    for (auto e : rrt_query -> tree_visualization)
                        tree_visualization.push_back(e);

                // std::cout << "Solution cost: " << rrt_query -> solution_cost << std::endl;
                //  Now we have a plan.
                if (rrt_query -> solution_traj.size() == 0)
                {
                    // std::cout << state_space -> print_point(rrt_query -> start_state,4) << std::endl;
                    // Apply the fallback for the next cycle.
                    std::cout << "No solution found, falling back." << std::endl;
                    rrt_spec -> stopping_control(rrt_query->start_state, safety_time);
                    rrt_query->solution_plan.append_onto_back(safety_time);
                    control_space -> copy_to_point(rrt_query -> solution_plan.back().control);
                    control_space -> enforce_bounds(rrt_query -> solution_plan.back().control);
                    std::cout << rrt_query -> solution_plan.print(4) << std::endl;
                    rrt_spec -> propagate(rrt_query -> start_state, rrt_query -> solution_plan, rrt_query -> solution_traj);
                }
                if (rrt_query -> solution_traj.size() <= planning_time*multiplier)
                {
                    state_space->copy_point(final_state, rrt_query->solution_traj.back());
                    if(!rrt_query->goal_check(final_state))
                    {
                        // prx_throw("This has not been dealt with.");
                        std::cout << "Solution too short. Falling back. " << rrt_query -> solution_traj.size() << std::endl;
                        safety_time = planning_time - rrt_query -> solution_cost;
                        rrt_spec -> stopping_control(final_state, safety_time);
                        rrt_query->solution_plan.append_onto_back(safety_time);
                        control_space -> copy_to_point(rrt_query -> solution_plan.back().control);
                        control_space -> enforce_bounds(rrt_query -> solution_plan.back().control);
                        rrt_query -> solution_traj.clear();
                        rrt_spec -> propagate(rrt_query -> start_state, rrt_query -> solution_plan, rrt_query -> solution_traj);
                    }
                }
                // std::cout << rrt_query -> solution_traj.print(4) << std::endl;
                // std::cout << rrt_query -> solution_plan.print(4) << std::endl;

                unsigned next_execution_index = std::min(planning_time*multiplier, (rrt_query -> solution_traj.size() - 1.0));
                state_space->copy_point(next_execution_state, rrt_query -> solution_traj[next_execution_index]);

                // We have to do this otherwise there may be duplicates.
                trajectory_t copy_traj(rrt_query -> solution_traj);
                state_space -> copy_point(final_state, next_execution_state);
                copy_traj.resize(next_execution_index);
                full_solution_trajectory += copy_traj;

                // Check if the current execution cycle would lead to a collision.
                // This part is going to executed no matter what, so if there's a collision here,
                // then the planner screwed up.
                for (unsigned i = 0; i <= next_execution_index && continue_planning; i++)
                {
                    sim -> update_all_obstacle_poses(rrt_query -> start_time + i * simulation_step);
                    state_space -> copy_point(step_state, rrt_query -> solution_traj[i]);
                    std::cout << "Checking state " << state_space->print_point(step_state,4) <<
                     " @ " << rrt_query -> start_time + i * simulation_step << std::endl;
                    if (!rrt_spec -> valid_state(step_state))
                        std::cout << "Collision during execution! t = " << rrt_query -> start_time + i * simulation_step << std::endl;
                    continue_planning &= rrt_spec -> valid_state(step_state);
                }

                // continue_planning &= !rrt_query -> goal_check(next_execution_state);
                continue_planning &= space_t::euclidean_2d(next_execution_state, global_goal_state) >= 0.5;

                // Update the planning info for the next planning cycle.
                sim -> update_all_obstacle_poses(rrt_query -> start_time);

                // Update the start state for the next planning cycle.
                state_space -> copy_point(rrt_query -> start_state, next_execution_state);
                rrt_query -> start_time += planning_time;

            } while (continue_planning && current_cycle <= max_replanning_cycles);
            full_solution_trajectory.copy_onto_back(final_state);
            planner -> reset();
        }
}