#include "prx/simulation/controllers/lqr.hpp"

namespace prx
{
	

	lqr_t::~lqr_t()
	{

	}

	
	void lqr_t::compute_K()
	{
		Eigen::MatrixXd A = lti -> get_A();
		Eigen::MatrixXd B = lti -> get_B();
		Eigen::MatrixXd S = care::solve(A, B, Q, R);
		K = R.inverse() * (B.transpose() * S);

	}

	void lqr_t::compute_controls()
	{
		prx_assert(K.rows() > 0 && K.cols() > 0, "Gain matrix (K) has not been computed! Call lqr_t -> compute_K() needed.");
		plant -> get_state_space() -> copy_to_vector(X);
		U = - K * (X - X_goal);
// std::cout << "X: " << X.transpose() << "\tU: " << U << std::endl;
		plant -> get_control_space() -> copy_from_vector(U);
		plant -> get_control_space() -> enforce_bounds();

	}

}