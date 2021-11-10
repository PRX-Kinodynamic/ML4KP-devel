#pragma once

#include "prx/planning/planners/planner.hpp"
#include "prx/planning/planner_functions/planner_functions.hpp"
#include "prx/utilities/data_structures/gnn.hpp"
#include "prx/utilities/data_structures/tree.hpp"
#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/timer.hpp"
#include "prx/utilities/data_structures/sigma.hpp"
#include "prx/planning/planners/rrt.hpp"

namespace prx
{
// struct Node {
// 		Point coords;
// 		double yaw;
// 		int index, parent;
// 		double cost, sigma, J;
// 		Mat cov;
// 	};

	class rrbt_node_t : public rrt_node_t
	{
		public:
			rrbt_node_t() : rrt_node_t()
			{
				sigma = 0;
			}

			void init_cov(int N)
			{
				cov = Eigen::MatrixXd::Identity(N,N);
			}
			virtual ~rrbt_node_t(){}

			double sigma;
			Eigen::MatrixXd cov;

	};

	class rrbt_specification_t : public rrt_specification_t
	{
	public:
		rrbt_specification_t(std::shared_ptr<system_group_t> sg,std::shared_ptr<collision_group_t> cg) 
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
		virtual ~rrbt_specification_t(){}

		steer_t steer;
		double steering_parameter;
		double radius;

	};

	class rrbt_query_t : public rrt_query_t
	{
	public:
		rrbt_query_t(space_t* state_space, space_t* control_space) : rrt_query_t(state_space,control_space)
		{
		}
		virtual ~rrbt_query_t(){}

	};

	class rrbt_t : public rrt_t
	{
	public:
		rrbt_t(const std::string& new_name);
		virtual ~rrbt_t();

	protected:

		// virtual void update_goal(node_index_t node_index);

		virtual void _link_and_setup_spec(planner_specification_t* spec) override;
		virtual bool _preprocess() override;
		virtual bool _link_and_setup_query(planner_query_t* query) override;
		virtual void _resolve_query(condition_check_t* condition) override;
		virtual void _fulfill_query() override;
		virtual void _reset() override;

		void propagate_belief(Eigen::MatrixXd &P1, space_point_t &qNear, space_point_t &qNew, double &sigma, Eigen::MatrixXd &P2);


		// virtual void bnb(node_index_t v, double cost_bound, bool delete_flag = false);

		rrbt_specification_t* rrbt_spec;
		rrbt_query_t* rrbt_query;
		steer_t steer;

		double steering_parameter;
		double radius;

		space_point_t dx; // Used for belief prop

		Eigen::VectorXd H;// H(1, DIM);

		Eigen::MatrixXd I; // = Eigen::MatrixXd::Identity(DIM, DIM);
		Eigen::MatrixXd A; // = I;
		Eigen::MatrixXd B; // = I;
		Eigen::MatrixXd G; // = I;

		Eigen::MatrixXd GQG;// = G * Q * G.transpose();
		Eigen::MatrixXd P_prd; //, P2;

		float processNoise = 0.028;
		Eigen::MatrixXd Q = pow(processNoise, 2) * I;
		double M = 1;
		double R, z, S, r;

		// Eigen::MatrixXd H = Eigen::MatrixXd(1, DIM);
		Eigen::MatrixXd K;

		// virtual void _link_and_setup_spec_shared(std::shared_ptr<planner_specification_t> spec) override;
		// virtual bool _link_and_setup_query_shared(std::shared_ptr<planner_query_t> query) override;

	};
}
