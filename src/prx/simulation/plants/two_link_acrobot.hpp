#pragma once

#include "prx/simulation/plant.hpp"
#include "prx/simulation/plants/types/linear_time_invariant.hpp"
#include "prx/simulation/plants/types/linear_time_variant.hpp"


namespace prx
{
	class two_link_acrobot_t : public plant_t
	{
	public:
		two_link_acrobot_t(const std::string& path);
		virtual ~two_link_acrobot_t();

		virtual void propagate(const double simulation_step) override final;

		virtual void update_configuration() override;

		// virtual bool linearize() override final;

		virtual bool linearize(Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::MatrixXd& C, Eigen::MatrixXd& D, space_point_t xt = nullptr, space_point_t ut = nullptr, double epsilon = 1e-3) override final;
        // bool linearize(space_point_t xt, space_point_t ut, double epsilon = 1e-3) override;

	protected:

		virtual void compute_derivative() override final;

		double _theta1,_theta2,_theta1dot,_theta2dot,_tau,_theta1dotdot,_theta2dotdot;

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

PRX_REGISTER_SYSTEM(two_link_acrobot_t, Acrobot)
