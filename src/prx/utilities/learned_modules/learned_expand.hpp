#pragma once 

#include "prx/utilities/learned_modules/learned_controller.hpp"
#include "prx/utilities/learned_modules/local_goal_predictor.hpp"
#include "prx/utilities/learned_modules/reachability_estimator.hpp"

namespace prx
{
    class learned_expand_t
    {
        public:
        learned_expand_t(planner_specification_t spec, planner_query_t query)
        {
            // planner_spec = &spec;
            // planner_query = &query;
        }

        void init(param_loader params)
        {
            controller = new learned_controller_t(params);
            lg_predictor = new local_goal_predictor_t(params);
            reachability_estimator = new reachability_estimator_t(params);
        }

        void expand(space_point_t& s, std::vector<plan_t*>& plans, std::vector<trajectory_t*>& trajs, int bn, bool blossom_expand)
        {
            plans.clear();
            trajs.clear();
            
        }

        learned_controller_t* controller;
        local_goal_predictor_t* lg_predictor;
        reachability_estimator_t* reachability_estimator;
    };
}