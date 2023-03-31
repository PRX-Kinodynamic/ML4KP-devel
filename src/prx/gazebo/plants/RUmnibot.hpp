#pragma once

#include "prx/simulation/plant.hpp"
#include "prx/simulation/plants/omnirobot_FO.hpp"
#include "prx/gazebo/plants/plant_gz_wrapper.hpp"

namespace prx
{
	class RUmnibot_t : public omnirobot_FO_t, public plant_gz_t
	{
	public:
		RUmnibot_t(const std::string& path)
			: plant_gz_t(), omnirobot_FO_t(path)
		{
			reset_vec = {0,0,0};
		}
		
		virtual ~RUmnibot_t();

		void copy_to_model_ptr() const override;

		void copy_from_model_ptr() override;

		void reset_system() override;

		// void set_wheel_angle(std::vector<double> _angles)

	protected:
		// virtual void compute_derivative() { pendulum_t::compute_derivative(); }

	};
}

PRX_REGISTER_SYSTEM_GZ(RUmnibot_t, RUmnibot)
