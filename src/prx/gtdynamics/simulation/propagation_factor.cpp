
#include "prx/gtdynamics/simulation/propagation_factor.hpp"

namespace prx
{

   	// gtsam::Key xt0_key, gtsam::Key xt1_key, gtsam::Key xdt1_key,
	Eigen::VectorXd propagation_factor_t::evaluateError(
			const X1& xt0, const X2& xt1, const X3& xdt1, 
			const X4& ut1,
      		boost::optional<Eigen::MatrixXd&> H1,
      		boost::optional<Eigen::MatrixXd&> H2,
      		boost::optional<Eigen::MatrixXd&> H3,
      		boost::optional<Eigen::MatrixXd&> H4) const
	{
		auto ss = ltv -> get_state_space();
		auto cs = ltv -> get_control_space();
		Eigen::VectorXd error(ss -> get_dimension());

		ss -> copy_from_vector(xt0);
		cs -> copy_from_vector(ut1);
		ltv -> compute_derivative();
		
		if (H1)
		{
			ss -> copy_to_point(xt);
			ltv -> linearize(xt, ut);
			*H1 = ltv -> get_A();
		}
		ltv -> get_derivative_space() -> copy_from_vector(xdt1);
		
		ltv -> propagate(0.1);
		ss -> copy_to_vector(error);
		error = error - xt1;

		if (H2)
		{
			ss -> copy_to_point(xt);
			ltv -> linearize(xt, ut);
			*H2 = ltv -> get_A();
		}

		if (H3)
		{
			*H3 = Eigen::MatrixXd::Zero(ss -> get_dimension(), ss -> get_dimension());
		}

		if (H4)
		{
			*H4 = Eigen::MatrixXd::Zero(ss -> get_dimension(), ss -> get_dimension());
		}

		return error;
	}

}