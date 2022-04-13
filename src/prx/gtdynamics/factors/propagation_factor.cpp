
#include "prx/gtdynamics/factors/propagation_factor.hpp"

namespace prx
{

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

		return error ;
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
			std::function<Eigen::VectorXd(Eigen::VectorXd)> fp = 
				std::bind(&propagation_factor_t::compute_error, this, 
							std::placeholders::_1, xt1, ut1);
   			*H1 = math_functions::differentiate(fp, xt0);
		}

		if (H2)
		{	
			std::function<Eigen::VectorXd(Eigen::VectorXd)> fp = 
				std::bind(&propagation_factor_t::compute_error, this, 
							xt0, std::placeholders::_1, ut1);
			*H2 = math_functions::differentiate(fp, xt1);
		}

		if (H3)
		{	
			std::function<Eigen::VectorXd(Eigen::VectorXd)> fp = 
				std::bind(&propagation_factor_t::compute_error, this, 
							xt0, xt1, std::placeholders::_1);
			*H3 = math_functions::differentiate(fp, ut1);
		}

		// std::cout << "error: " << error.transpose() << std::endl;
		return error ;

	}

		Eigen::VectorXd propagation_factor_4_t::compute_error(
		Eigen::VectorXd xt0, Eigen::VectorXd xt1, 
		Eigen::VectorXd ut1, Eigen::VectorXd t01) const
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

		ltv -> propagate(t01[0]);
		ss -> copy_to_point(error_pt);
		ss -> copy_point_from_vector(xt, xt1);
		ss -> copy_to_vector(dbg);

		ss -> difference(error_pt, xt, error_pt);
		ss -> copy_vector_from_point(error, error_pt);

		return error;
	}

	Eigen::VectorXd propagation_factor_4_t::evaluateError(
			const X1& xt0, const X2& xt1, 
			const X3& ut1, const X4& t01,
      		boost::optional<Eigen::MatrixXd&> H1,
      		boost::optional<Eigen::MatrixXd&> H2,
      		boost::optional<Eigen::MatrixXd&> H3,
      		boost::optional<Eigen::MatrixXd&> H4) const
	{
		auto error = compute_error(xt0, xt1, ut1, t01);
		if (H1)
		{	
			std::function<Eigen::VectorXd(Eigen::VectorXd)> fp = 
				std::bind(&propagation_factor_4_t::compute_error, this, 
							std::placeholders::_1, xt1, ut1, t01);
   			*H1 = math_functions::differentiate(fp, xt0);
		}

		if (H2)
		{	
			std::function<Eigen::VectorXd(Eigen::VectorXd)> fp = 
				std::bind(&propagation_factor_4_t::compute_error, this, 
							xt0, std::placeholders::_1, ut1, t01);
			*H2 = math_functions::differentiate(fp, xt1);
		}

		if (H3)
		{	
			std::function<Eigen::VectorXd(Eigen::VectorXd)> fp = 
				std::bind(&propagation_factor_4_t::compute_error, this, 
							xt0, xt1, std::placeholders::_1, t01);
			*H3 = math_functions::differentiate(fp, ut1);
		}

		if (H4)
		{	
			std::function<Eigen::VectorXd(Eigen::VectorXd)> fp = 
				std::bind(&propagation_factor_4_t::compute_error, this, 
							xt0, xt1, ut1, std::placeholders::_1);
			*H4 = math_functions::differentiate(fp, ut1);
		}

		// std::cout << "error: " << error.transpose() << std::endl;
		return error;

	}

}