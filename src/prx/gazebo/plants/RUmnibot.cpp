#include "prx/gazebo/plants/RUmnibot.hpp"

namespace prx
{

	RUmnibot_t::~RUmnibot_t()
	{}

	void RUmnibot_t::copy_to_model_ptr() const
	{
		// PRX_DEBUG_ITERABLE("GetJoints", )
		// for (auto model : m_ptr -> GetJoints())
		// {
		// 	std::cout << "Joint: " << model -> GetName() << std::endl;
		// }
		// std::cout << 
		m_ptr -> GetJoint("joint_chassis_omniwheel_1") -> SetVelocity(0, w1);
		m_ptr -> GetJoint("joint_chassis_omniwheel_2") -> SetVelocity(0, w2);
		m_ptr -> GetJoint("joint_chassis_omniwheel_3") -> SetVelocity(0, w3);
		m_ptr -> GetJoint("joint_chassis_omniwheel_4") -> SetVelocity(0, w4);
	}

	void RUmnibot_t::copy_from_model_ptr()
	{
		auto pose = m_ptr -> WorldPose();
        x = pose.Pos().X();
        y = pose.Pos().Y();
        theta = pose.Rot().Euler().Z();
	}

	void RUmnibot_t::reset_system()
	{
		state_space -> copy_from_vector(reset_vec);
		copy_to_model_ptr();
	}

	// void RUmnibot_t::set_wheel_angle(std::vector<double> _angles)
	// {
	// 	m_ptr -> GetJoint("joint_chassis_omniwheel_1") -> SetAngle(0, math::Angle{ _angles[0] });
	// 	m_ptr -> GetJoint("joint_chassis_omniwheel_2") -> SetAngle(0, math::Angle{ _angles[1] });
	// 	m_ptr -> GetJoint("joint_chassis_omniwheel_3") -> SetAngle(0, math::Angle{ _angles[2] });
	// 	m_ptr -> GetJoint("joint_chassis_omniwheel_4") -> SetAngle(0, math::Angle{ _angles[3] });
	// }

}