#pragma once

#include "prx/simulation/plant.hpp"
#include "prx/simulation/systems/linear_time_invariant.hpp"

namespace prx
{
	class pendulum_t : public plant_t, lti_t
	{
	public:
		pendulum_t(const std::string& path);
		virtual ~pendulum_t();

		virtual void propagate(const double simulation_step) override final;

		virtual void update_configuration() override;

	protected:

		virtual void compute_derivative() override final;

		double _theta1,_theta2,_theta1dot,_theta2dot,_tau,_theta1dotdot,_theta2dotdot;

		double gravity = 9.81;
        double length = 0.5;
        double friction = 0.1;
        double inertia;
        double mass = 0.15;

	};
}

PRX_REGISTER_SYSTEM(pendulum_t, pendulum)
