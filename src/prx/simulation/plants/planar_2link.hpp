#pragma once

#include "prx/simulation/plant.hpp"
#include "prx/simulation/plants/types/linear_time_invariant.hpp"
#include "prx/simulation/plants/types/linear_time_variant.hpp"


namespace prx
{
	class planar_2link_t : public ltv_t
	{
	public:
		planar_2link_t(const std::string& path);
		virtual ~planar_2link_t();

		virtual void propagate(const double simulation_step) override final;

		virtual void update_configuration() override;

		// virtual bool linearize() override final;

        bool linearize(space_point_t xt, space_point_t ut, double epsilon = 1e-3) override;

        virtual double kinetic_energy() override;

		virtual double potential_energy() override;

		// virtual void compute_control(space_point_t u) override;
		virtual Eigen::MatrixXd get_mass_matrix() override;
		virtual Eigen::VectorXd get_coriolis_vector() override;
		virtual Eigen::VectorXd get_gravity_vector() override;

	protected:

		virtual void compute_derivative() override final;

		double _theta1,_theta2,_theta1dot,_theta2dot,_theta1dotdot,_theta2dotdot;
		double _tau_0;
		double _tau_1;

        double mass = 1.0;
        double g = 9.81;
        double l1 = 1.0;
        double l2 = 1.0;
        double I1 = 0.2;
        double I2 = 1.0;
        double d1 = 1.0; // Damping
        double d2 = 1.0; 

        double viz_length = 20;

	};
}

PRX_REGISTER_SYSTEM(planar_2link_t, planar_2link)
