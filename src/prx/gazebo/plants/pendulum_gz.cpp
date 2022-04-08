#include "prx/gazebo/plants/pendulum_gz.hpp"

namespace prx
{

	pendulum_gz_t::~pendulum_gz_t()
	{}

	void pendulum_gz_t::copy_to_model_ptr() const
	{
		m_ptr -> GetJoints()[0]  -> SetPosition(0, _theta1, false);
        m_ptr -> GetJoints()[0] -> SetVelocity(0, _theta1dot);
        m_ptr -> Update();
	}

	void pendulum_gz_t::copy_from_model_ptr()
	{
		_theta1    = m_ptr -> GetJoints()[0] -> WorldPose().Rot().Euler().X();
        _theta1dot = m_ptr -> GetJoints()[0] -> GetVelocity(0);
	}

	void pendulum_gz_t::reset_system()
	{
		state_space -> copy_from_vector(reset_vec);
		copy_to_model_ptr();
	}


}