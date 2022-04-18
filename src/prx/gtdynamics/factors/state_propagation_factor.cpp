
#include "prx/gtdynamics/factors/state_propagation_factor.hpp"

namespace prx
{

	Eigen::VectorXd state_propagation_factor_t::compute_error(
		Eigen::VectorXd xt0, Eigen::VectorXd xt1, Eigen::VectorXd ut1) const
	{
		auto ss = ltv -> get_state_space();
		auto cs = ltv -> get_control_space();
		auto ss_dim = ss -> get_dimension();
		auto cs_dim = cs -> get_dimension();
		// Eigen::VectorXd error = Eigen::VectorXd::Zero(ss_dim);
		// Eigen::VectorXd dbg = Eigen::VectorXd::Zero(ss_dim);

		ss -> copy_from_vector(xt0);
		ss -> enforce_bounds();
		cs -> copy_from_vector(ut1);
		cs -> enforce_bounds();
		ltv -> compute_derivative();

		ltv -> propagate(simulation_step);
		ss -> copy_to_point(error_pt);
		ss -> copy_point_from_vector(xt, xt1);

		ss -> difference(error_pt, error_pt, xt);
		// ss -> copy_vector_from_point(error, error_pt);

		n_vector_t<1> err(1);
		err[0] = error_pt -> at(dim_i);
		return err ;
	}

   	// gtsam::Key xt0_key, gtsam::Key xt1_key, gtsam::Key xdt1_key,
	Eigen::VectorXd state_propagation_factor_t::evaluateError(
			const X1& xt0, const X2& xt1, 
			const X3& ut1,
      		boost::optional<Eigen::MatrixXd&> H1,
      		boost::optional<Eigen::MatrixXd&> H2,
      		boost::optional<Eigen::MatrixXd&> H3) const
	{
		auto error = compute_error(xt0, xt1, ut1);
		if (H1)
		{
			std::function<Eigen::VectorXd(Eigen::VectorXd)> fp = 
				std::bind(&state_propagation_factor_t::compute_error, this, 
							std::placeholders::_1, xt1, ut1);
   			*H1 = math_functions::differentiate(fp, xt0);
		}

		if (H2)
		{	
			std::function<Eigen::VectorXd(Eigen::VectorXd)> fp = 
				std::bind(&state_propagation_factor_t::compute_error, this, 
							xt0, std::placeholders::_1, ut1);
			*H2 = math_functions::differentiate(fp, xt1);
		}

		if (H3)
		{	
			std::function<Eigen::VectorXd(Eigen::VectorXd)> fp = 
				std::bind(&state_propagation_factor_t::compute_error, this, 
							xt0, xt1, std::placeholders::_1);
			*H3 = math_functions::differentiate(fp, ut1);
		}

		// std::cout << "error: " << error.transpose() << std::endl;
		return error ;

	}

}