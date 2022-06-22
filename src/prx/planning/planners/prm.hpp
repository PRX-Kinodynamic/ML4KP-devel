#pragma once

#include "prx/planning/planners/planner.hpp"
#include "prx/planning/planner_functions/planner_functions.hpp"
#include "prx/utilities/data_structures/gnn.hpp"
#include "prx/utilities/data_structures/undirected_graph.hpp"
#include "prx/utilities/data_structures/abstract_node.hpp"
#include "prx/utilities/data_structures/abstract_edge.hpp"
#include "prx/utilities/defs.hpp"

#define PLANNER_NAME "PRM"

namespace prx
{
    class prm_node_t : public undirected_node_t
    {
        public:
            prm_node_t() : undirected_node_t(){}
            virtual ~prm_node_t(){}
    };
    class prm_edge_t : public undirected_edge_t
    {
        public:
            prm_edge_t() : undirected_edge_t() {}
            virtual ~prm_edge_t(){}
    };

    class prm_specification_t : public planner_specification_t
    {
        public:
        prm_specification_t(std::shared_ptr<system_group_t> sg, std::shared_ptr<collision_group_t> cg)
        {
            state_space = sg->get_state_space();
            control_space = sg->get_control_space();

            cost_function = [](const trajectory_t& t, const plan_t& plan)
            {
                // TODO
                return default_cost_function(t,plan);
            };
            distance_function = [](const space_point_t& s1, const space_point_t& s2)
            {
                return space_t::euclidean_2d(s1, s2);
            };
            valid_state = [this,cg](space_point_t& s)
			{
				return default_valid_state(s, state_space,cg);
			};
			valid_check = [this,cg](trajectory_t& traj)
			{
				return default_valid_trajectory(traj, state_space,cg);
			};
            sample_state = [this](space_point_t& s)
            {
                return default_sample_state(s,state_space);
            };
            local_planner = [this](space_point_t& s1, space_point_t& s2, trajectory_t& traj, unsigned size)
            {
                return default_interpolate_trajectory(s1, s2, traj, size);
            };
        }
        virtual ~prm_specification_t(){}

        cost_function_t cost_function;
        distance_function_t distance_function;
        valid_state_t valid_state;
        valid_trajectory_t valid_check;
        sample_state_t sample_state;
        interpolate_trajectory_t local_planner;
        int k, M;

		space_t* state_space;
        space_t* control_space;
    };
    class prm_query_t : public planner_query_t
    {
        public:
        prm_query_t(space_t* state_space, space_t* control_space) : planner_query_t(state_space,control_space)
        {
            clear_outputs();
        }
        virtual ~prm_query_t(){}
    };

    class prm_t : public planner_t
    {
        public:
            prm_t(const std::string& new_name);
            virtual ~prm_t();
            double get_closest_cost(const space_point_t& s);
        protected:
            virtual void _link_and_setup_spec(planner_specification_t* spec) override;
            virtual bool _preprocess() override;
            virtual bool _link_and_setup_query(planner_query_t* query) override;
            virtual void _resolve_query(condition_check_t* condition) override;
            virtual void _fulfill_query() override;
            virtual void _reset() override;

            prm_specification_t* prm_spec;
            prm_query_t* prm_query;

            std::string planner_name;

            cost_function_t cost_function;
            distance_function_t distance_function;
            valid_state_t valid_state;
            valid_trajectory_t valid_check;
            sample_state_t sample_state;
            interpolate_trajectory_t local_planner;

            space_point_t sample_point;

            graph_nearest_neighbors_t* metric;
            undirected_graph_t graph;

            int k, M;

            space_t* state_space;
            space_t* control_space;
            
            node_index_t start_vertex;
            node_index_t goal_vertex;
    };
}
