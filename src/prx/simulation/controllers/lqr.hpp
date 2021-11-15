#pragma once

#include "prx/simulation/controller.hpp"
#include "prx/simulation/plants/types/linear_time_invariant.hpp"
#include "prx/utilities/math/continuous_algebraic_riccati_equation.hpp"

namespace prx
{
	class lqr_t : public controller_t
	{
		public:
		template<class S>
		lqr_t(std::shared_ptr<S> _plant, std::string _name) 
		: controller_t(_plant, _name)
		{
			lti = std::dynamic_pointer_cast<lti_t>(_plant);
			prx_assert(lti != nullptr, "Plant is not an lti_t!");
		}
		template<class S>
		lqr_t(std::shared_ptr<S> _plant, Eigen::MatrixXd _Q, Eigen::MatrixXd _R, std::string _name)
			: controller_t(_plant, _name)
		{
			lti = std::dynamic_pointer_cast<lti_t>(_plant);
			prx_assert(lti != nullptr, "Plant is not an lti_t!");
			set_Q(_Q);
			set_R(_R);
		}
		void set_Q(Eigen::MatrixXd _Q)
		{
			Q = _Q;
		}

		void set_R(Eigen::MatrixXd _R)
		{
			R = _R;
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

			std::shared_ptr<lti_t> lti;
	};
}