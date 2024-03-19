#pragma once

#include "prx/utilities/defs.hpp"
#include "prx/utilities/data_structures/gnn.hpp"
#include "prx/utilities/data_structures/undirected_graph.hpp"
#include "prx/planning/planners/planner.hpp"
#include "prx/planning/planner_functions/planner_functions.hpp"

namespace prx
{
    class roadmap_t : public planner_t
    {
        public:
            roadmap_t(const std::string& new_name) : planner_t(new_name)
            {
                metric = nullptr;
                planner_name = new_name;
            }
            virtual ~roadmap_t(){
            }

            virtual void _link_and_setup_spec(planner_specification_t* spec) override;
            virtual bool _preprocess() override;
            virtual bool _link_and_setup_query(planner_query_t* query) override;
            virtual void _resolve_query(condition_check_t* condition) override;
            virtual void _fulfill_query() override;
            virtual void _reset() override;

        protected:
            void msmo_astar();
            void interpolate(space_point_t&, space_point_t&, std::vector<space_point_t>&);

            std::string planner_name;

            node_index_t start_vertex;
            node_index_t goal_vertex;

            distance_function_t distance_function;
            valid_state_t valid_state;
            sample_state_t sample_state;
            // interpolate_t interpolate; 
            // std::function<void(space_point_t&, space_point_t&, std::vector<space_point_t>&)>

            undirected_graph_t roadmap;
            graph_nearest_neighbors_t* metric;

            space_t* state_space;
            int index;

            int neighbor_radius;



    };
}