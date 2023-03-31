#pragma once

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>


#include "prx/simulation/plant.hpp"
#include "prx/simulation/playback/plan.hpp"
#include "prx_plan.pb.h"

namespace prx
{
	typedef const boost::shared_ptr<const prx_gz::msgs::prx_plan>
    	prx_plan_ptr;
	class execute_plan_plugin_t : public gazebo::ModelPlugin
	{
		public:

			execute_plan_plugin_t();
			virtual ~execute_plan_plugin_t();

			virtual void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

		protected:

        	// Gazebo-related stuff 
    		gazebo::transport::NodePtr node;
    		gazebo::transport::SubscriberPtr sub;
    		gazebo::physics::ModelPtr model;
	    	gazebo::physics::JointPtr joint;

		private: 
		
			void process_message(prx_plan_ptr &_msg)
    		{
    		  // this->SetVelocity(_msg->x());
    		}
	};

	GZ_REGISTER_MODEL_PLUGIN(execute_plan_plugin_t)
}

