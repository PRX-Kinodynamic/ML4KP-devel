#pragma once

#include "prx/simulation/plant.hpp"
#include "prx/simulation/plants/types/linear_time_invariant.hpp"

namespace prx
{
	class pendulum_t : public lti_t
	{
	public:
		pendulum_t(const std::string& path);
		virtual ~pendulum_t();

		virtual void propagate(const double simulation_step) override final;

		virtual void update_configuration() override;

		virtual bool linearize() override final;


	protected:

		inline
		void compute_inertia()
		{
			inertia = mass * length * length;
		}

		virtual void compute_derivative() override final;

		double _theta1,_theta2,_theta1dot,_theta2dot,_tau,_theta1dotdot,_theta2dotdot;

		const double gravity = 9.81;
        double length = 0.5;
        double friction = 0.1;
        double inertia;
        double mass = 0.15;
        double normalize = true;

        Eigen::DiagonalMatrix<double,2> Tx, Tx_inv;
        Eigen::DiagonalMatrix<double,1> Tu, Tu_inv;

	};
}

PRX_REGISTER_SYSTEM(pendulum_t, pendulum)
