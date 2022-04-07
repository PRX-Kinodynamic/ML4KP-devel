#include "prx/gazebo/plants/pendulum_gz.hpp"

namespace prx
{
	// pendulum_gz_t::pendulum_gz_t(const std::string& path)
	// 	// : plant_gz_wrapper_t(path),  plant_gz_t(path)
	// 	: plant_gz_wrapper_t(path)
	// 	// : pendulum_t(path),  plant_gz_t(path)
	// 	// : plant_gz_t(path)
	// {}

	pendulum_gz_t::~pendulum_gz_t()
	{}

	void pendulum_gz_t::copy_to_model_ptr() const
	// void pendulum_gz_t::copy_to_model_ptr(const gazebo::physics::ModelPtr& m_ptr) const
	{
        // std::cout << "space state: " << state_space -> print_memory(3) << std::endl;
        // std::cout << "state: " << _theta1 << " " << _theta1dot << std::endl;
		auto p = m_ptr -> GetLinks()[0] -> WorldPose();
		ignition::math::Vector3<double> vn(p.Pos().X(), p.Pos().Y(), p.Pos().Z());
		// ignition::math::Vector3<double> vn(0,0,1.5);
		ignition::math::Quaternion<double> qn(_theta1, p.Rot().Euler().Y(), p.Rot().Euler().Z());
		ignition::math::Pose3<double> pn(vn, qn);//(m_ptr -> GetLinks()[0] -> WorldPose());
		// pn.Rot().Euler().X(_theta1);
        // std::cout << "pose: " << pn << std::endl;
		m_ptr -> GetLinks()[0] -> SetWorldPose(pn);
        m_ptr -> GetJoints()[0] -> SetVelocity(0, _theta1dot);

        // std::cout << "model pose: " << m_ptr -> GetLinks()[0] -> WorldPose() << std::endl;
        // std::cout << "model joint: " << m_ptr -> GetJoints()[0] << std::endl;

		
	}

	// void pendulum_gz_t::copy_from_model_ptr(const gazebo::physics::ModelPtr& m_ptr)
	void pendulum_gz_t::copy_from_model_ptr()
	{
                // std::cout << "Link Pose: " << m -> GetLinks()[0] -> WorldPose() << std::endl;
                // std::cout << "Joint Vel: " << m -> GetJoints()[0] -> GetVelocity(0) << std::endl;
		_theta1    = m_ptr -> GetLinks()[0] -> WorldPose().Rot().Euler().X();
        _theta1dot = m_ptr -> GetJoints()[0] -> GetVelocity(0);
	}
}