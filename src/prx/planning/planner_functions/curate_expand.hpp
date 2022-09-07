#pragma once
#include "prx/utilities/defs.hpp"
#include "prx/utilities/learned_modules/learned_controller.hpp"
#include "prx/planning/planner_functions/planner_functions.hpp"
#include "prx/planning/planners/dirt.hpp"

using namespace prx;

class curate_expand_t
{
    public:
    curate_expand_t(dirt_specification_t spec, dirt_query_t query, std::shared_ptr<system_group_t> _sg)
    {
        // planner_spec = &spec;
        // planner_query = &query;
        goal_state = spec.state_space -> clone_point(query.goal_state);

        sample_plan = spec.sample_plan;
        propagate = spec.propagate;
        valid_trajectory = spec.valid_check;
        h = spec.h;

        sg = _sg;
        sampled_state = sg -> get_state_space() -> make_point();
    }

    void init(param_loader params)
    {
        controller = new learned_controller_t(params);

        control_duration = params["/learned_controller/control_duration"].as<double>();
    }

    void expand(space_point_t& s, std::vector<plan_t*>& plans, std::vector<trajectory_t*>& trajs, int bn, bool blossom_expand)
    {
        prx_assert(bn == 1, "blossom_number should be 1 for curate_expand");

        plans.clear();
        trajs.clear();

        std::vector<std::vector<double>> current_states;
        std::vector<std::vector<double>> sampled_goals;

        std::vector<double> current_state;
        sg -> get_state_space() -> copy_vector_from_point(current_state, s);
        std::vector<double> sampled_goal;

        for (int i = 0; i < bn; i++)
        {
            sampled_goal.clear();
            sg -> get_state_space() -> sample(sampled_state);
            sg -> get_state_space() -> copy_vector_from_point(sampled_goal, sampled_state);
            sampled_goals.push_back(sampled_goal);
            current_states.push_back(current_state);
        }

        auto controls = controller -> get_controls(current_states,sampled_goals);

        trajectory_t traj(sg -> get_state_space()), best_traj(sg -> get_state_space());
        plan_t plan(sg -> get_control_space()), best_plan(sg -> get_control_space());
        double best_h = PRX_INFINITY;

        for (int i = 0; i < bn; i++)
        {
            traj.clear(); plan.clear();
            plan.append_onto_back(control_duration);
            if (uniform_random() < 0.5)
                sg -> get_control_space() -> copy_point_from_vector(plan.back().control,controls[i]);
            else
                sg -> get_control_space() -> sample(plan.back().control);
            propagate(s,plan,traj);
            if (!valid_trajectory(traj) || traj.size() == 0) continue;
            
            double current_h = h(traj.back(),goal_state);
            if (current_h > best_h) continue;

            best_h = current_h;
            best_traj.clear(); best_plan.clear();
            best_traj += traj;
            best_plan += plan;
        }

        if (best_traj.size() > 0)
        {
            plans.push_back(new plan_t(best_plan));
            trajs.push_back(new trajectory_t(best_traj));
        }
    }
    
    learned_controller_t* controller;

    std::shared_ptr<system_group_t> sg;
    sample_plan_t sample_plan;
    propagate_t propagate;
    valid_trajectory_t valid_trajectory;
    heuristic_function_t h;

    space_point_t sampled_state, goal_state;

    int blossom_number = 1000;
    double control_duration;
};