#include "prx/planning/replanners/replanner.hpp"

namespace prx
{

        replanner_t::replanner_t(const std::string& name) 
        {
            planner_name = name;
            continue_planning = true;
            current_cycle = -1;
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
            do
            {
                // Increment the cycle and update the underlying planner's horizon.
                current_cycle += 1;
                rrt_spec -> horizon = (current_cycle + 1) * horizon;

                // Perform the planning cycle.
                perform_single_planning_cycle();

                //  Now we have a plan.
                std::cout << rrt_query -> solution_cost << std::endl;

                continue_planning = false;

            } while (continue_planning);
            
        }
}