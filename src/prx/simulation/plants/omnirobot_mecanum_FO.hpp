#pragma once

#include "prx/simulation/plant.hpp"
#include "prx/simulation/playback/trajectory.hpp"
#include "prx/simulation/playback/plan.hpp"

namespace prx
{
	/**
	 * @brief <b> A first-order omnidirectional vehicle. </b>
	 * 
	 * @author Zakary Littlefield, Aravind Sivaramakrishnan, Edgar Granados, Seth Karten
	 * */
	class omnirobot_mecanum_FO_t : public plant_t
	{
		public:
		omnirobot_mecanum_FO_t(const std::string& path);

		virtual ~omnirobot_mecanum_FO_t();

		virtual void propagate(const double simulation_step) override final;

		virtual void update_configuration() override;

		virtual void compute_derivative() override final;

		bool connect_points(space_point_t origin, space_point_t local_goal, plan_t& plan, trajectory_t& traj, double dist_limit = std::numeric_limits<double>::infinity(), double tolerance = 0.1);

		// bool connect_points(space_point_t origin, space_point_t local_goal, trajectory_t& traj);

		template<typename T>
		void set_parameter(std::string p_name, T p_val) 
		{
			if (p_name == "mu" && std::is_same<T, std::vector<double>>::value)
			{
				mu[0] = p_val[0];
				mu[1] = p_val[1];
				mu[2] = p_val[2];
				mu[3] = p_val[3];
			}
			else
			{
				prx_warn("omnirobot_FO_t::set_parameter - No '" << p_name << "' parameter");
			}
		}

		template<typename T>
		T get_parameter(std::string p_name) 
		{
			T res;
			if (p_name == "mu" && std::is_same<T, std::vector<double>>::value)
			{
				res = {mu[0], mu[1], mu[2], mu[3]};
			}
			else
			{
				prx_warn("omnirobot_FO_t::get_parameter - No '" << p_name << "' parameter");
			}
			return res;
		}


	protected:
		double x,y,theta;
		double x_d, y_d, theta_d;
		double w1, w2, w3, w4;
		double R = 0.03;

		const double motor_rpm = (330.0 * 2.0 * PRX_PI)/60.0;
		const double mass = 4;
		const double gravity = 9.8;
		const double radius = 0.03;
		const double stall_torque = 0.6;

		// Assuming all angles between wheels are equal
		Eigen::MatrixXd D; // Velocity coupling matrix
		Eigen::MatrixXd Dp; // D+ -> Pseudoinverse of D
		Eigen::Vector4d U;
		Eigen::Vector3d Xd;
		Eigen::Vector4d mu;
		double mu_0, mu_1, mu_2, mu_3;
		// Eigen::Matrix3d w1_r;
		transform_t w1_tr;
		// transform_t w2_tr;

		const double l_a = .11;
		const double l_b = .10;
		const double l_ab = l_a + l_b;

	private:
		std::vector<double> lower_bound = {-11,-11,-M_PI};
		std::vector<double> upper_bound = {11,11,M_PI};

		space_point_t state;
		space_point_t ctrl;
	};


}
PRX_REGISTER_SYSTEM(omnirobot_mecanum_FO_t, omnirobot_mecanum_FO)

auto or_vel_fn = [](prx::system_ptr_t sys_ptr)
{

    double vel = 0;
	return vel;
};
PRX_REGISTER_VELOCITY_FN(omnirobot_mecanum_FO_t, or_vel_fn)
