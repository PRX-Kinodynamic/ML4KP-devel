#pragma once

#include "prx/planning/planners/planner.hpp"
#include "prx/planning/planner_functions/planner_functions.hpp"
#include "prx/utilities/data_structures/gnn.hpp"
#include "prx/utilities/data_structures/tree.hpp"
#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/timer.hpp"
#include "prx/utilities/data_structures/sigma.hpp"
#include "prx/planning/planners/rrt.hpp"

// #define PLANNER_NAME "RRT*"
namespace prx
{

	class rrt_star_specification_t : public rrt_specification_t
	{
	public:
		rrt_star_specification_t(std::shared_ptr<system_group_t> sg,std::shared_ptr<collision_group_t> cg) 
			: rrt_specification_t(sg, cg)
		{
			steer = [](space_point_t& state, space_point_t& local_goal, plan_t& plan, trajectory_t& traj)
			{
				prx_throw("rrt_star_specification_t.steer_t: No steer function! ");
				return false;
			};
			steering_parameter = 1;
			radius = 1;
		}
		virtual ~rrt_star_specification_t(){}

		steer_t steer;
		double steering_parameter;
		double radius;

	};

	class rrt_star_query_t : public rrt_query_t
	{
	public:
		rrt_star_query_t(space_t* state_space, space_t* control_space) : rrt_query_t(state_space,control_space)
		{
		}
		virtual ~rrt_star_query_t(){}

	};

	class rrt_star_t : public rrt_t
	{
	public:
		rrt_star_t(const std::string& new_name);
		virtual ~rrt_star_t();

		// virtual void print_statistics();

		// virtual std::vector<std::string> get_statistics_header() override;
		// virtual std::vector<double> get_statistics() override;
	protected:

		// virtual void update_goal(node_index_t node_index);

		virtual void _link_and_setup_spec(planner_specification_t* spec) override;
		virtual bool _preprocess() override;
		virtual bool _link_and_setup_query(planner_query_t* query) override;
		virtual void _resolve_query(condition_check_t* condition) override;
		virtual void _fulfill_query() override;
		virtual void _reset() override;

		// virtual void bnb(node_index_t v, double cost_bound, bool delete_flag = false);

		rrt_star_specification_t* rrt_star_spec;
		rrt_star_query_t* rrt_star_query;
		steer_t steer;

		double steering_parameter;
		double radius;

		// virtual void _link_and_setup_spec_shared(std::shared_ptr<planner_specification_t> spec) override;
		// virtual bool _link_and_setup_query_shared(std::shared_ptr<planner_query_t> query) override;

	};
}
