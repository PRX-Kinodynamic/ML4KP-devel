#pragma once

#include "prx/simulation/plant.hpp"
#include "prx/simulation/plants/pendulum.hpp"
#include "prx/gazebo/plants/plant_gz.hpp"

namespace prx
{
	class pendulum_gz_t : public pendulum_t, plant_gz_t
	{
	public:
		pendulum_gz_t(const std::string& path);
		
		virtual ~pendulum_gz_t();

		virtual gazebo::physics::ModelState get_model_state() const override;

		virtual void set_model_state(const gazebo::physics::ModelState&) override;

		// virtual void update_configuration() override { pendulum_t::update_configuration(); } ;


	protected:
		// virtual void compute_derivative() { pendulum_t::compute_derivative(); }

	};
}

// PRX_REGISTER_SYSTEM(pendulum_gz_t, pendulum_gz)
