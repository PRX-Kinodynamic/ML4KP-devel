#pragma once

#include "prx/utilities/defs.hpp"
#include "prx/utilities/data_structures/gnn.hpp"
#include "prx/utilities/data_structures/undirected_graph.hpp"
#include "prx/planning/planners/planner.hpp"
#include "prx/planning/planner_functions/planner_functions.hpp"

namespace prx
{   
    class roadmap_node_t : public undirected_node_t
    {
    public:
    roadmap_node_t()
    {
        cost_to_come = 0;
    }
    virtual ~roadmap_node_t()
    {
    }
        double cost_to_come;

    };

    class roadmap_edge_t : public undirected_edge_t
    {
    public:
        roadmap_edge_t()
        {
            edge_cost = 0;
            num_collisions = 0;
        }
        virtual ~roadmap_edge_t()
        {
        }

    double edge_cost;
    unsigned short num_collisions;
    };

    class roadmap_specification_t : public planner_specification_t
    {
        public:
            roadmap_specification_t(std::shared_ptr<system_group_t> sg, std::shared_ptr<collision_group_t> cg)
            {
                _sg = sg;
                state_space = sg->get_state_space();

                distance_function = [](const space_point_t& s1, const space_point_t& s2) { return space_t::euclidean_2d(s1, s2); };
                sample_state = [this](space_point_t& s) { default_sample_state(s, state_space); };

                valid_state = [this, cg](space_point_t& s) { return default_valid_state(s, state_space, cg); };
            }
            virtual ~roadmap_specification_t()
            {
            }

            std::shared_ptr<system_group_t> _sg;

            distance_function_t distance_function;
            valid_state_t valid_state;
            sample_state_t sample_state;

            space_t* state_space;

    };

    class roadmap_query_t : public planner_query_t
    {
        public:
            // control space will not be used in roadmap query
            roadmap_query_t(space_t* state_space, space_t* control_space) : planner_query_t(state_space, control_space)
            {
                clear_outputs();

                goal_region_radius = 0.5;

                goal_check = [&](space_point_t s) { return default_goal_check(s, goal_state, goal_region_radius); };
            }
            virtual ~roadmap_query_t()
            {
            }

            double goal_region_radius;
    };

    class roadmap_t : public planner_t
    {
        public:
            roadmap_t(const std::string& new_name);
            virtual ~roadmap_t();

            virtual void _link_and_setup_spec(planner_specification_t* spec) override;
            virtual bool _preprocess() override;
            virtual bool _link_and_setup_query(planner_query_t* query) override;
            virtual void _resolve_query(condition_check_t* condition) override;
            virtual void _fulfill_query() override;
            virtual void _reset() override;

            roadmap_specification_t* roadmap_spec;
            roadmap_query_t* roadmap_query;

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

            space_point_t sample_point;

            int neighbor_radius;

            long unsigned iteration_count;
            timer_t timer;

            double current_solution;
            long unsigned current_solution_iters;
            double current_solution_time;

            int print_statistics_count;

    };
}