#pragma once

#include "prx/utilities/defs.hpp"
#include "prx/planning/planners/dirt_replan.hpp"
#include "prx/simulation/playback/plan.hpp"
#include "prx/simulation/playback/trajectory.hpp"
#include "prx/planning/condition_check.hpp"
#include "prx/utilities/general/param_loader.hpp"

namespace prx
{
    typedef std::function<std::vector<double> (const space_point_t&, const std::vector<double>&)> waypoint_function_t;
    class replanner_t
    {
    public:
        replanner_t(const std::string& name,dirt_replan_specification_t* spec);
        ~replanner_t();
        void setup(param_loader params);
        void link_world_model(std::shared_ptr<world_model_t> wm);
        void link_planner(dirt_replan_t* planner, dirt_replan_specification_t* spec, dirt_replan_query_t* query);
        void resolve_query();

        void reset()
        {
            continue_planning = true;
            current_cycle = -1;
            tree_visualization.clear();
        }

        void perform_single_planning_cycle()
        {
            rrt_query -> clear_outputs();
            // planner -> reset();
            checker -> reset();

            planner -> link_and_setup_spec(rrt_spec);
            planner -> preprocess();
            planner -> link_and_setup_query(rrt_query);

            planner -> resolve_query(checker);
            planner -> fulfill_query();
        }

        std::string planner_name;
        trajectory_t full_solution_trajectory;

        waypoint_function_t waypoint_function;

        std::vector<trajectory_t> tree_visualization;
    protected:
        dirt_replan_t* planner;
        dirt_replan_specification_t* rrt_spec;
        dirt_replan_query_t* rrt_query;
        std::shared_ptr<world_model_t> sim;
        space_t* state_space;
        space_t* control_space;
        space_point_t global_goal_state;


    private:
        /**
         * @brief This is the allowed planning time for a single cycle.
         * 
         */
        double planning_time;

        /**
         * @brief This is the maximum number of replanning cycles to be performed.
         * 
         */
        int max_replanning_cycles;

        /**
         * @brief Maximum duration of the replanned trajectory.
         * 
         */
        double horizon;

        /**
         * @brief A condition checker.
         * 
         */
        condition_check_t* checker;

        /**
         * @brief The current replanning cycle.
         * 
         */
        int current_cycle;

        /**
         * @brief Whether to continue planning or not
         * 
         */
        bool continue_planning;
    };
}