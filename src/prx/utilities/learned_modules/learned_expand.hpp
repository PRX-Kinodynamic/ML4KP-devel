#pragma once 

#include "prx/utilities/learned_modules/learned_controller.hpp"
#include "prx/utilities/learned_modules/local_goal_predictor.hpp"
#include "prx/utilities/learned_modules/reachability_estimator.hpp"

namespace prx
{
    class learned_expand_t
    {
        public:
        learned_expand_t(rrt_specification_t spec, rrt_query_t query, std::shared_ptr<system_group_t> _sg)
        {
            // planner_spec = &spec;
            // planner_query = &query;
            goal_state = spec.state_space -> clone_point(query.goal_state);
            spec.state_space -> copy_vector_from_point(goal_state_vec,goal_state);

            sample_plan = spec.sample_plan;
            propagate = spec.propagate;

            sg = _sg;
        }

        void init(param_loader params)
        {
            controller = new learned_controller_t(params);
            lg_predictor = new local_goal_predictor_t(params);
            reachability_estimator = new reachability_estimator_t(params);

            control_duration = params["/learned_controller/control_duration"].as<double>();
        }

        void expand(space_point_t& s, std::vector<plan_t*>& plans, std::vector<trajectory_t*>& trajs, int bn, bool blossom_expand)
        {
            plans.clear();
            trajs.clear();

            if (blossom_expand)
            {
                sg->get_state_space()->copy_vector_from_point(current_state_vec,s);

                // Get local goal prediction.
                std::vector<double> lg_prediction = goal_state_vec;
                if (!reachability_estimator->is_reachable(current_state_vec,goal_state_vec))
                {
                    lg_prediction = lg_predictor->get_local_goal(current_state_vec, goal_state_vec);
                }

                // Get controller prediction
                std::vector<double> controller_prediction = controller->get_control(current_state_vec, lg_prediction);
                
                trajectory_t traj(sg->get_state_space());
                plan_t plan(sg->get_control_space());

                plan.append_onto_back(control_duration);
                sg->get_control_space()->copy_point_from_vector(plan.back().control,controller_prediction);

                sg->propagate(s,plan,traj);

                plans.push_back(new plan_t(plan));
                trajs.push_back(new trajectory_t(traj));
            }

            else
            {
                default_expand(s,plans,trajs,bn,sg,sample_plan,propagate);
            }
        }

        learned_controller_t* controller;
        local_goal_predictor_t* lg_predictor;
        reachability_estimator_t* reachability_estimator;

        double control_duration;

        std::shared_ptr<system_group_t> sg;
        sample_plan_t sample_plan;
        propagate_t propagate;

        space_point_t goal_state;
        std::vector<double> goal_state_vec, current_state_vec;
    };
}