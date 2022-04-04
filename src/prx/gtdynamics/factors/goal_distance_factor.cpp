
#include "prx/gtdynamics/factors/goal_distance_factor.hpp"

namespace prx
{

	

   	// gtsam::Key xt0_key, gtsam::Key xt1_key, gtsam::Key xdt1_key,
	Eigen::VectorXd goal_distance_factor_t::evaluateError(
			const X& xt_v, 
      		boost::optional<Eigen::MatrixXd&> H1) const
	{	
		auto ss = system_ptr -> get_state_space();
		auto ss_dim = ss -> get_dimension();
		Eigen::VectorXd error = Eigen::VectorXd::Zero(ss_dim);

		ss -> copy_point_from_vector(xt_pt, xt_v);
		// ss -> copy_point_from_vector(xt_goal_pt, xt_goal_v);

		ss -> difference(xt_pt, xt_goal_pt, error_pt);
		ss -> copy_vector_from_point(error, error_pt);

		if (H1)
		{
			*H1 = Eigen::MatrixXd::Identity(1,1);
			// *H1 = Eigen::MatrixXd::Identity(ss -> get_dimension(), ss -> get_dimension());
		}
		// std::cout << "xt_pt: " << xt_pt << std::endl;
		// std::cout << "xt_goal_pt: " << xt_goal_pt << std::endl;
		// std::cout << "error: " << error;
		// for (int i = 0; i < ss_dim; ++i)
		// {	
			// if (std::fabs(error[i]) <= 0.01) continue;
			// error[i] = std::exp(error[i])-1.0;
			// error[i] = std::pow(error[i], t_i);
		// }
		Eigen::MatrixXd Q = 1 * Eigen::MatrixXd::Identity(ss -> get_dimension(), ss -> get_dimension());
		// Q(0,0) = 10;
		// Q(1,1) = 10;
		// std::cout << "\terror: " << error.transpose() * Q * error << std::endl;
		return error.transpose() * Q * error;
	}

}