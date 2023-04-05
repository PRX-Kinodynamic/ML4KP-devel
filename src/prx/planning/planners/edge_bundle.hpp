#pragma once

#include "prx/planning/planners/planner.hpp"
#include "prx/planning/planner_functions/planner_functions.hpp"
#include "prx/utilities/data_structures/gnn.hpp"
#include "prx/utilities/data_structures/tree.hpp"
#include "prx/utilities/data_structures/open_set.hpp"
#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/timer.hpp"

namespace prx
{
    class edge_bundle_node_t : public abstract_node_t
    {
        public:
            edge_bundle_node_t() {}
            ~edge_bundle_node_t() {}            

            node_index_t get_index() const
            {
                return index;
            }

            void set_index(node_index_t i)
            {
                index = i;
            }

            std::shared_ptr<trajectory_t> traj;
            std::shared_ptr<plan_t> plan;
        
        protected:
            node_index_t index;
    };

    class edge_bundle_planner_node_t : public tree_node_t
    {
        public:
            edge_bundle_planner_node_t()
            {
                cost_to_come = 0;
                cost_to_go = PRX_INFINITY;
            }
            virtual ~edge_bundle_planner_node_t(){}

            double cost_to_come, cost_to_go;

            astar_node_t* astar_node;
    };

    class edge_bundle_planner_edge_t : public tree_edge_t
    {
        public:
            edge_bundle_planner_edge_t()
            {
                edge_cost = 0;
            }
            virtual ~edge_bundle_planner_edge_t(){}

            std::shared_ptr<plan_t> plan;
            std::shared_ptr<trajectory_t> traj;
            double edge_cost;
    };

    class edge_bundle_planner_specification_t : public planner_specification_t
    {
        public:
        edge_bundle_planner_specification_t(std::shared_ptr<system_group_t> sg,std::shared_ptr<collision_group_t> cg)
        {
            state_space = sg->get_state_space();
            control_space = sg->get_control_space();
            double multiplier = simulation_step >= 1 ? simulation_step : 1./simulation_step;
			min_control_steps = 0.5*multiplier;
			max_control_steps = 2.0*multiplier; 
            num_edges = int(1e5);
            neighborhood_radius = 0.5;

            cost_function = [](const trajectory_t& t, const plan_t& plan)
			{
				return default_cost_function(t,plan);
			};
			distance_function = [](const space_point_t& s1, const space_point_t& s2)
			{
				return space_t::euclidean_2d(s1, s2);
			};
			sample_state = [this](space_point_t& s)
			{
				default_sample_state(s,state_space);
			};
			sample_plan = [this](plan_t& p, space_point_t pose)
			{
				default_sample_plan(p,control_space,min_control_steps,max_control_steps);
			};
            valid_state = [this,cg](space_point_t& s)
			{
				return default_valid_state(s, state_space,cg);
			};
            valid_check = [this,cg](trajectory_t& traj)
			{
				return default_valid_trajectory(traj, state_space,cg);
			};
            propagate = [sg](space_point_t& start_state, plan_t& plan, trajectory_t& out_traj)
			{
				default_propagate(start_state,plan,out_traj,sg);
			};
            h = [this](const space_point_t& s, const space_point_t& s2)
			{
				return default_heuristic_function(s, s2, distance_function);
			};
        }

        virtual ~edge_bundle_planner_specification_t(){}

        cost_function_t cost_function;
		distance_function_t distance_function;
		sample_state_t sample_state;
		sample_plan_t sample_plan;
		valid_trajectory_t valid_check;
        valid_state_t valid_state;
		propagate_t propagate;
        heuristic_function_t h;

        space_t* state_space;
        space_t* control_space;

        int min_control_steps;
        int max_control_steps;
        unsigned long num_edges;
        double neighborhood_radius;
    };

    class edge_bundle_planner_query_t : public planner_query_t
    {
        public:
        edge_bundle_planner_query_t(space_t* state_space, space_t* control_space) : planner_query_t(state_space,control_space)
        {
            clear_outputs();

			goal_region_radius = 0.5;

			goal_check = [&](space_point_t s)
			{
				return default_goal_check(s, goal_state, goal_region_radius);
			};
        }

        virtual ~edge_bundle_planner_query_t(){}
        double goal_region_radius;

    };

    class edge_bundle_planner_t : public planner_t 
    {
        public:
        edge_bundle_planner_t(const std::string& new_name);
        virtual ~edge_bundle_planner_t();

        protected:
        virtual void update_goal(node_index_t node_index, condition_check_t* condition);

		virtual void _link_and_setup_spec(planner_specification_t* spec) override;
		virtual bool _preprocess() override;
		virtual bool _link_and_setup_query(planner_query_t* query) override;
		virtual void _resolve_query(condition_check_t* condition) override;
		virtual void _fulfill_query() override;
		virtual void _reset() override;
        virtual void bnb(node_index_t v, double cost_bound, bool delete_flag = false);
        
        edge_bundle_planner_specification_t* spec;
        edge_bundle_planner_query_t* query;

        open_set_t open_set;

        private:
        cost_function_t cost_function;
		distance_function_t distance_function;
		sample_state_t sample_state;
		sample_plan_t sample_plan;
		valid_trajectory_t valid_check;
        valid_state_t valid_state;
		propagate_t propagate;
        heuristic_function_t h;

        tree_t tree;
        graph_nearest_neighbors_t* metric;

        long unsigned num_edges;
        std::unordered_map<node_index_t, edge_bundle_node_t*> bundle;
        double neighborhood_radius;

        std::string planner_name;
        node_index_t start_vertex;
        node_index_t goal_vertex;

        space_t* state_space;
		space_t* control_space;

        space_point_t sample_point;
        bool child_extension;
        node_index_t previous_child;

		long unsigned iteration_count;
		timer_t timer;

		double current_solution;
		long unsigned current_solution_iters;
		double current_solution_time;
		double current_solution_sim_time;
    };
}