#include "prx/simulation/controllers/lqr.hpp"

namespace prx
{
	

	lqr_t::~lqr_t()
	{

	}

	
	void lqr_t::compute_K()
	{
		auto A = lti -> get_A();
		auto B = lti -> get_B();
		auto S = care::solve(A, B, Q, R);
		K = R.inverse() * (B.transpose() * S);
		X.resize(lti -> get_state_space() -> get_dimension());
		U.resize(lti -> get_control_space() -> get_dimension());
	}

	void lqr_t::compute_controls()
	{
		prx_assert(K.rows() > 0 && K.cols() > 0, "Gain matrix (K) has not been computed! Call lqr_t -> compute_K() needed.");
		plant -> get_state_space() -> copy_to_vector(X);
		U = - K * X;
// std::cout << "U: " << U << std::endl;
		plant -> get_control_space() -> copy_from_vector(U);
		plant -> get_control_space() -> enforce_bounds();

	}

}