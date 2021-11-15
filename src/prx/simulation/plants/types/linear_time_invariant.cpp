#include <unsupported/Eigen/MatrixFunctions>
#include "prx/simulation/plants/types/linear_time_invariant.hpp"

namespace prx
{
	lti_t::lti_t(std::string _path) : plant_t(_path)
	{

	}

	lti_t::~lti_t()
	{
	}

	bool lti_t::check()
	{
		prx_assert(lti_stt_space != nullptr, "lti_stt_space has not been set");
		prx_assert(lti_ctr_space != nullptr, "lti_ctr_space has not been set");
		double n = lti_stt_space -> get_dimension();
		double p = lti_ctr_space -> get_dimension();

		prx_assert(A.rows() == n && A.cols() == n, "Matrix A has wrong dimensions");
		prx_assert(B.rows() == n && B.cols() == p, "Matrix B has wrong dimensions");
		prx_assert(C.cols() == n, "Matrix C has wrong dimensions"); 
		prx_assert(D.cols() == p, "Matrix D has wrong dimensions"); 
		prx_assert(C.rows() == D.rows(), "Matrix C or D have wrong dimensions"); 

		return true;
	}

	void lti_t::derivative()
	{
		lti_stt_space -> copy_to_vector(x);
		lti_ctr_space -> copy_to_vector(u);

		xd = A * x + B * u;

		lti_stt_space -> copy_from_vector(xd);

	}

	Eigen::VectorXd lti_t::derivative_and_output()
	{
		derivative();
		return C * x + D * u;
	}

	void lti_t::discretize()
	{
PRX_DEBUG_PRINT
		double n = lti_stt_space -> get_dimension();
		double p = lti_ctr_space -> get_dimension();

PRX_DEBUG_PRINT
		Eigen::MatrixXd AB;
PRX_DEBUG_PRINT
		AB = Eigen::MatrixXd::Zero(n+p, n+p);
PRX_DEBUG_PRINT
		AB.block(0,0,n,n) = A;
PRX_DEBUG_PRINT
		AB.block(0,n,n,p) = B;
		AB *= simulation_step;
PRX_DEBUG_PRINT
		Eigen::MatrixXd AB_res = AB.exp();
		std::cout << "e^AB: " << AB_res << std::endl;
		A = AB_res.block(0,0,n,n);
		B = AB_res.block(0,n,n,p);
	}


}