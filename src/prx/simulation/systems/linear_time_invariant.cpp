#include "prx/simulation/systems/linear_time_invariant.hpp"

namespace prx
{
	lti_t::lti_t(space_t* _state_space, space_t* _control_space)
	{
		stt_space = _state_space;
		ctr_space = _control_space;
	}

	lti_t::~lti_t()
	{
	}

	bool lti_t::check()
	{
		double n = stt_space -> get_dimension();
		double p = ctr_space -> get_dimension();

		if (A.rows() != n || A.cols() != n)	return false;
		if (B.rows() != n || B.cols() != p)	return false;
		if (C.cols() != n) return false;
		if (D.cols() != p) return false;
		if (C.rows() != D.rows()) return false;

		return true;
	}

	void lti_t::derivative()
	{
		stt_space -> copy_to_vector(x);
		ctr_space -> copy_to_vector(u);

		xd = A * x + B * u;

		stt_space -> copy_from_vector(xd);

	}

	Eigen::VectorXd lti_t::derivative_and_output()
	{
		derivative();
		return C * x + D * u;
	}

}