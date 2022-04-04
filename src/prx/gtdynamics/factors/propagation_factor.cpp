
#include "prx/gtdynamics/factors/propagation_factor.hpp"

namespace prx
{

	Eigen::MatrixXd differentiate(
		std::function<Eigen::VectorXd(Eigen::VectorXd)> f,
	 	Eigen::VectorXd xt)
	{
		auto epsilon = std::sqrt(simulation_step);
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
// mat.col(mat.cols()-1) = vec;
		}

		return diff;
	}

	Eigen::VectorXd propagation_factor_t::compute_error(
		Eigen::VectorXd xt0, Eigen::VectorXd xt1, Eigen::VectorXd ut1) const
	{
		auto ss = ltv -> get_state_space();
		auto cs = ltv -> get_control_space();
		auto ss_dim = ss -> get_dimension();
		auto cs_dim = cs -> get_dimension();
		Eigen::VectorXd error = Eigen::VectorXd::Zero(ss_dim);
		Eigen::VectorXd dbg = Eigen::VectorXd::Zero(ss_dim);

		ss -> copy_from_vector(xt0);
		ss -> enforce_bounds();
		cs -> copy_from_vector(ut1);
		cs -> enforce_bounds();
		ltv -> compute_derivative();

		ltv -> propagate(simulation_step);
		ss -> copy_to_point(error_pt);
		ss -> copy_point_from_vector(xt, xt1);
		ss -> copy_to_vector(dbg);

		ss -> difference(error_pt, xt, error_pt);
		ss -> copy_vector_from_point(error, error_pt);

		return error;
	}

   	// gtsam::Key xt0_key, gtsam::Key xt1_key, gtsam::Key xdt1_key,
	Eigen::VectorXd propagation_factor_t::evaluateError(
			const X1& xt0, const X2& xt1, 
			const X3& ut1,
      		boost::optional<Eigen::MatrixXd&> H1,
      		boost::optional<Eigen::MatrixXd&> H2,
      		boost::optional<Eigen::MatrixXd&> H3) const
	{
		auto error = compute_error(xt0, xt1, ut1);
		if (H1)
		{	
			std::function<Eigen::VectorXd(Eigen::VectorXd)> fp = std::bind(&propagation_factor_t::compute_error, this, std::placeholders::_1, xt1, ut1);
   			*H1 = differentiate(fp, xt0);
		}

		if (H2)
		{	
			std::function<Eigen::VectorXd(Eigen::VectorXd)> fp = std::bind(&propagation_factor_t::compute_error, this, xt0, std::placeholders::_1, ut1);
			*H2 = differentiate(fp, xt1);
		}

		if (H3)
		{	
			std::function<Eigen::VectorXd(Eigen::VectorXd)> fp = std::bind(&propagation_factor_t::compute_error, this, xt0, xt1, std::placeholders::_1);
			*H3 = differentiate(fp, ut1);
		}

		// std::cout << "error: " << error.transpose() << std::endl;
		return error;
		// auto ss = ltv -> get_state_space();
		// auto cs = ltv -> get_control_space();
		// auto ss_dim = ss -> get_dimension();
		// auto cs_dim = cs -> get_dimension();
		// Eigen::VectorXd error = Eigen::VectorXd::Zero(ss_dim);
		// Eigen::VectorXd dbg = Eigen::VectorXd::Zero(ss_dim);

		// ss -> copy_from_vector(xt0);
		// // std::cout << "pre: " << ss -> print_memory(4);
		// ss -> enforce_bounds();
		// // std::cout << "\tpos: " << ss -> print_memory(4) << std::endl;
		// cs -> copy_from_vector(ut1);
		// cs -> enforce_bounds();
		// ltv -> compute_derivative();
		
		// if (H1)
		// {
		// 	ss -> copy_to_point(xt);
		// 	ltv -> linearize(xt, ut);
		// 	// *H1 += ltv -> get_A();
		// 	*H1 = ltv -> get_A() + Eigen::MatrixXd::Identity(ss_dim, ss_dim);
		// }
		// ltv -> propagate(simulation_step);
		// ss -> copy_to_point(error_pt);
		// ss -> copy_point_from_vector(xt, xt1);
		// ss -> copy_to_vector(dbg);

		// ss -> difference(error_pt, xt, error_pt);
		// ss -> copy_vector_from_point(error, error_pt);

		// if (H2)
		// {

		// 	*H2 = -Eigen::MatrixXd::Identity(ss_dim, ss_dim);
		// }

		// if (H3)
		// {
		// 	ss -> copy_point_from_vector(xt, xt0);
		// 	ltv -> linearize(xt, ut);
		// 	*H3 = Eigen::MatrixXd::Zero(ss_dim, ss_dim);
		// 	Eigen::MatrixXd B = ltv -> get_B();

		// 	for (int i = 0; i < cs_dim ; ++i)
		// 	{
		// 		(*H3).col(i) = B ;
		// 		// (*H3).row(i) = B.transpose();
		// 	}
		// }

		// return error * 1e1;
	}

}