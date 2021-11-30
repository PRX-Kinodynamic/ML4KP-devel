#pragma once

#include "prx/simulation/controller.hpp"
#include "prx/simulation/plants/types/linear_time_invariant.hpp"
#include "prx/utilities/math/continuous_algebraic_riccati_equation.hpp"

namespace prx
{
	class lqr_t : public controller_t
	{
		public:
		// template<class S>
		lqr_t(std::shared_ptr<lti_t> _plant, std::string _name) 
		: controller_t(_plant, _name)
		{
			lti = std::dynamic_pointer_cast<lti_t>(_plant);
			prx_assert(lti != nullptr, "Plant is not an lti_t!");
			int n = lti -> get_state_space() -> get_dimension();
			int m = lti -> get_control_space() -> get_dimension();
			X.resize(n);
			U.resize(m);
			X_goal.resize(n);
			X_goal = Eigen::VectorXd::Zero(n);
		}
		// template<class S>
		lqr_t(std::shared_ptr<lti_t> _plant, Eigen::MatrixXd _Q, Eigen::MatrixXd _R, std::string _name)
			: controller_t(_plant, _name)
		{
			lti = std::dynamic_pointer_cast<lti_t>(_plant);
			prx_assert(lti != nullptr, "Plant is not an lti_t!");
			set_Q(_Q);
			set_R(_R);
			int n = lti -> get_state_space() -> get_dimension();
			int m = lti -> get_control_space() -> get_dimension();
			X.resize(n);
			U.resize(m);

			X_goal.resize(n);
			X_goal = Eigen::VectorXd::Zero(n);
		}
		void set_Q(Eigen::MatrixXd _Q)
		{
			Q = _Q;
		}

		void set_R(Eigen::MatrixXd _R)
		{
			R = _R;
		}

		void set_goal(Eigen::VectorXd _goal)
		{
			X_goal = _goal;
		}

		virtual ~lqr_t();

		virtual void compute_controls() override;

		void compute_K();

		Eigen::MatrixXd get_K() 
		{
			return K;
		}

		protected:
			Eigen::MatrixXd K;
			Eigen::MatrixXd Q;
			Eigen::MatrixXd R;

			Eigen::VectorXd X;
			Eigen::VectorXd U;

			Eigen::VectorXd X_goal;

			std::shared_ptr<lti_t> lti;
	};
}