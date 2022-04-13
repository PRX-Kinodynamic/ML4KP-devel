#pragma once

#include "prx/simulation/system.hpp"
#include "prx/utilities/general/constants.hpp"
#include "prx/utilities/general/prx_assert.hpp"
namespace prx
{
	namespace math_functions 
	{
		static Eigen::MatrixXd differentiate(
			std::function<Eigen::VectorXd(Eigen::VectorXd)> f,
	 		Eigen::VectorXd xt, double sim_step = simulation_step)
		{
			auto epsilon = std::sqrt(sim_step);
			auto x_dim = xt.size();
			Eigen::MatrixXd diff;
			Eigen::VectorXd x_plus = Eigen::VectorXd::Zero(xt.size());
			Eigen::VectorXd x_minus = Eigen::VectorXd::Zero(xt.size());

			for (int i=0; i < x_dim; i++) 
  			{
				x_plus  = xt;
				x_minus = xt;
				x_plus(i) += epsilon;
				x_minus(i) -= epsilon;

				auto f_plus = f(x_plus);
				auto f_minus = f(x_minus);

    			diff.conservativeResize(f_plus.rows(), i+1);
    			diff.col(i) = ( f_plus - f_minus) / ( 2. * epsilon );
			}

			return diff;
		}

	}
}