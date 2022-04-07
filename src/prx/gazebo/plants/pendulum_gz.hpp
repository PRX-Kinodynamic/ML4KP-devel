#pragma once

#include "prx/simulation/plant.hpp"
#include "prx/simulation/plants/pendulum.hpp"
#include "prx/gazebo/plants/plant_gz_wrapper.hpp"

namespace prx
{
	// class pendulum_gz_t : public plant_gz_wrapper_t<pendulum_t>, public plant_gz_t
	// class pendulum_gz_t : public plant_gz_wrapper_t<pendulum_t>
	// class pendulum_gz_t : public pendulum_t, public plant_gz_t
	class pendulum_gz_t : public pendulum_t, public plant_gz_t
	{
	public:
		pendulum_gz_t(const std::string& path)
			: plant_gz_t(), pendulum_t(path)
		{}
		
		virtual ~pendulum_gz_t();

		void copy_to_model_ptr() const override;
		// void copy_to_model_ptr(const gazebo::physics::ModelPtr&) const override;

		void copy_from_model_ptr() override;
		// void copy_from_model_ptr(const gazebo::physics::ModelPtr&) override;

		// virtual void update_configuration() override { pendulum_t::update_configuration(); } ;


	protected:
		// virtual void compute_derivative() { pendulum_t::compute_derivative(); }

	};
}

PRX_REGISTER_SYSTEM_GZ(pendulum_gz_t, pendulum_gz)
