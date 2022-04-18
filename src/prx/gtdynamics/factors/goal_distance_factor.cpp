
#include "prx/gtdynamics/factors/goal_distance_factor.hpp"

namespace prx
{

	Eigen::VectorXd goal_distance_factor_t::compute_error(
		Eigen::VectorXd xt_v) const
	{
		auto ss = system_ptr -> get_state_space();
		auto ss_dim = ss -> get_dimension();
		Eigen::VectorXd error = Eigen::VectorXd::Zero(ss_dim);

		ss -> copy_point_from_vector(xt_pt, xt_v);
		

		ss -> difference(xt_pt, xt_goal_pt, error_pt);
		ss -> copy_vector_from_point(error, error_pt);

		for (int i = 0; i < ss_dim; ++i)
		{
			error[i] *= error_scale[i];
			error[i] *= std::pow(theta, T-t_i);
		}
		// Eigen::MatrixXd Q = 1 * Eigen::MatrixXd::Identity(ss -> get_dimension(), ss -> get_dimension());
		// Q(0,0) = 10;
		// Q(1,1) = 10;
		// std::cout << "\terror: " << error.transpose() * Q * error << std::endl;
		// return error.transpose() * Q * error;
		return  error;
	}

	Eigen::VectorXd goal_distance_factor_t::evaluateError(
			const X& xt_v, 
      		boost::optional<Eigen::MatrixXd&> H1) const
	{	
		

		if (H1)
		{
			std::function<Eigen::VectorXd(Eigen::VectorXd)> fp = 
				std::bind(&goal_distance_factor_t::compute_error, this, 
							std::placeholders::_1);
   			*H1 = math_functions::differentiate(fp, xt_v);
		}

		return compute_error(xt_v);
		
	}

}