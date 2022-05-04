#pragma once

#include "prx/simulation/plant.hpp"
#include "prx/simulation/plants/types/linear_time_variant.hpp"

namespace prx
{
	class lander_LD_t : public plant_t
	{
	public:
		lander_LD_t(const std::string& path);
		virtual ~lander_LD_t();

		virtual void propagate(const double simulation_step) override final;

		virtual void update_configuration() override;

    	// virtual bool linearize(space_point_t xt, space_point_t ut, double epsilon) override;


	protected:

		virtual void compute_derivative() override final;

		// double _theta1,_theta2,_theta1dot,_theta2dot,_tau,_theta1dotdot,_theta2dotdot;
		double h, h_dot, h_ddot;
		double v, v_dot;
		double m, m_dot;
		double alpha;

		// Mass of the system when no fuel is left
		double m_s = 1; 
		double fuel_0 = 0.5; 

 		double gravity = 1.62;
 		double k = 0.1; // velocity of the exhaust gases with respect to the spacecraft
 		double h_0 = 5;

	};
}

PRX_REGISTER_SYSTEM(lander_LD_t, lander_LD)
