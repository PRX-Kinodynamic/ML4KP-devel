#pragma once

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

namespace prx
{
	class plant_gz_t 
	{

		public:
			plant_gz_t() = default;
			virtual ~plant_gz_t() = default;

			virtual gazebo::physics::ModelState get_model_state() const = 0;

			virtual void set_model_state(const gazebo::physics::ModelState&) = 0;

		private:

			gazebo::physics::ModelPtr model_ptr;
	};
}
