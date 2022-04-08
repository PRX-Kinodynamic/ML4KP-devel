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
		// auto p = m_ptr -> NestedModel("base") -> GetLinks()[0] -> WorldPose();
		// auto p = m_ptr -> GetJoints()[0] -> WorldPose();
		// ignition::math::Vector3<double> vn(p.Pos().X(), p.Pos().Y(), p.Pos().Z());
		// ignition::math::Vector3<double> an(_theta1dot,0,0);
		// // ignition::math::Vector3<double> vn(0,0,1.5);
		// ignition::math::Quaternion<double> qn(_theta1, p.Rot().Euler().Y(), p.Rot().Euler().Z());
		// ignition::math::Pose3<double> pn(vn, qn);//(m_ptr -> GetLinks()[0] -> WorldPose());
		// pn.Rot().Euler().X(_theta1);
        // std::cout << "pose: " << pn << std::endl;
		m_ptr -> GetJoints()[0]  -> SetPosition(0, _theta1, false);
		// m_ptr -> NestedModel("base") -> GetLinks()[0] -> SetWorldPose(pn);
		// for (auto l : m_ptr -> NestedModel("base") -> GetLinks())
		// {
  //       	std::cout << "link name: " << l -> GetName() << std::endl;
		// }
        m_ptr -> GetJoints()[0] -> SetVelocity(0, _theta1dot);
		// m_ptr -> GetJoints()[0] -> SetParam("vel", 0, _theta1dot);
		// m_ptr -> GetJoints()[0] -> SetParam("max_force", 0, 100);
		// m_ptr -> GetLinks()[0] -> SetAngularVel(an);
        // std::cout << "_theta1dot: " << _theta1dot << std::endl;
        // SetAngularVel <=== CHECK
        // m_ptr -> GetJoint("joint_00") -> SetVelocity(0, _theta1dot);
        // m_ptr -> GetJoint("joint_00") -> SetVelocityMaximal(0, _theta1dot);
        // SetVelocityMaximal
        // m_ptr -> GetJoints()[0] -> Update();
        m_ptr -> Update();
        // std::cout << "model pose: " << m_ptr -> GetLinks()[0] -> WorldPose() << std::endl;
        // std::cout << "vel  joint: " << m_ptr -> GetJoints()[0] -> GetVelocity(0) << std::endl;
        // std::cout << "vel  limit: " << m_ptr -> GetJoints()[0] -> GetVelocityLimit(0) << std::endl;

		
	}

	// void pendulum_gz_t::copy_from_model_ptr(const gazebo::physics::ModelPtr& m_ptr)
	void pendulum_gz_t::copy_from_model_ptr()
	{
                // std::cout << "Link Pose: " << m -> GetLinks()[0] -> WorldPose() << std::endl;
                // std::cout << "Joint Vel: " << m -> GetJoints()[0] -> GetVelocity(0) << std::endl;
		_theta1    = m_ptr -> GetJoints()[0] -> WorldPose().Rot().Euler().X();
        _theta1dot = m_ptr -> GetJoints()[0] -> GetVelocity(0);
        // std::cout << "space state: " << state_space -> print_memory(3) << std::endl;

	}

	void pendulum_gz_t::reset_system()
	{
		state_space -> copy_from_vector(reset_vec);
		copy_to_model_ptr();
	}


}