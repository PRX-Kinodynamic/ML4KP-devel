#pragma once

#include "prx/planning/planners/planner.hpp"
#include "prx/planning/planner_functions/planner_functions.hpp"
#include "prx/utilities/data_structures/gnn.hpp"
#include "prx/utilities/data_structures/graph.hpp"
#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/timer.hpp"
#include "prx/utilities/data_structures/undirected_graph.hpp"

#define PLANNER_NAME "RRT"
namespace prx
{
	class rrg_node_t : public undirected_node_t
	{
		public:

		rrg_node_t()
		{
			cost_to_come = 0;
			duration = 0;
			prev_set = false;
		}
		virtual ~rrg_node_t(){}

		double cost_to_come;
		double duration;
		node_index_t prev;
		bool prev_set;
	};

	class rrg_edge_t : public undirected_edge_t
	{
	public:
		rrg_edge_t()
		{
			edge_cost = 0;
		}
		virtual ~rrg_edge_t(){}

		std::shared_ptr<plan_t> plan;
		std::shared_ptr<trajectory_t> traj;
		double edge_cost;
	};

	class rrg_specification_t : public planner_specification_t
	{
	public:
		rrg_specification_t(std::shared_ptr<system_group_t> sg,std::shared_ptr<collision_group_t> cg)
		{
			bnb = false;
			state_space = sg->get_state_space();
			control_space = sg->get_control_space();
			double multiplier = simulation_step >= 1 ? simulation_step : 1./simulation_step;
			min_control_steps = 0.5*multiplier;
			max_control_steps = 5*multiplier;

			k_rrg = std::exp(1. + 1. / (state_space -> get_dimension()) );
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
				// default_sample_plan(p,control_space,100,400);
				// Changed to time
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
			steer = [](space_point_t& state, space_point_t& local_goal, plan_t& plan, trajectory_t& traj)
			{
				prx_throw("rrg_specification_t.steer_t: No steer function! ");
				return false;
			};

		}
		virtual ~rrg_specification_t(){}

		cost_function_t cost_function;
		distance_function_t distance_function;
		sample_state_t sample_state;
		sample_plan_t sample_plan;
		valid_trajectory_t valid_check;
		valid_state_t valid_state;
		steer_t steer;


		space_t* state_space;
		space_t* control_space;

		bool bnb;
		bool use_replanning;

		int min_control_steps;
		int max_control_steps;
		int blossom_number;
		double k_rrg;

	};

	class rrg_query_t : public planner_query_t
	{
	public:
		rrg_query_t(space_t* state_space, space_t* control_space) : planner_query_t(state_space,control_space)
		{
			clear_outputs();

			goal_check = [&](space_point_t s)
			{
				return space_t::euclidean_2d(s, goal_state) < goal_region_radius;
			};

		}
		virtual ~rrg_query_t(){}
		double goal_region_radius;
	};

	class rrg_t : public planner_t
	{
	public:
		rrg_t(const std::string& new_name);
		virtual ~rrg_t();

		virtual void print_statistics();

		virtual std::vector<std::string> get_statistics_header() override;
		virtual std::vector<double> get_statistics() override;
	protected:

		virtual void update_goal(node_index_t node_index);

		virtual void _link_and_setup_spec(planner_specification_t* spec) override;
		virtual bool _preprocess() override;
		virtual bool _link_and_setup_query(planner_query_t* query) override;
		virtual void _resolve_query(condition_check_t* condition) override;
		virtual void _fulfill_query() override;
		virtual void _reset() override;


		virtual void bnb(node_index_t v, double cost_bound, bool delete_flag = false);

		rrg_specification_t* rrg_spec;
		rrg_query_t* rrg_query;

		// virtual void _link_and_setup_spec_shared(std::shared_ptr<planner_specification_t> spec) override;
		// virtual bool _link_and_setup_query_shared(std::shared_ptr<planner_query_t> query) override;

		std::string planner_name;
		
		node_index_t start_vertex;
		node_index_t goal_vertex;

		distance_function_t distance_function;
		cost_function_t cost_function;
		sample_state_t sample_state;
		sample_plan_t sample_plan;
		valid_trajectory_t valid_check;
		steer_t steer;


		undirected_graph_t graph;
		graph_nearest_neighbors_t* metric;

		space_t* state_space;
		space_t* control_space;

		space_point_t sample_point;

		long unsigned iteration_count;
		timer_t timer;

		double current_solution;
		long unsigned current_solution_iters;
		double current_solution_time;

		int print_statistics_count;

		double k_rrg;

	};
}
