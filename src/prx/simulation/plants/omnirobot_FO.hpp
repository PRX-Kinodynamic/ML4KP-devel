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
	class omnirobot_FO_t : public plant_t
	{
		public:
		omnirobot_FO_t(const std::string& path);

		virtual ~omnirobot_FO_t();

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
		double R;

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
		// Eigen::Matrix3d w1_r;
		transform_t w1_tr;
		// transform_t w2_tr;

	private:
		std::vector<double> lower_bound = {-11,-11,-3.15};
		std::vector<double> upper_bound = {11,11,3.15};

		space_point_t state;
		space_point_t ctrl;
	};


}
PRX_REGISTER_SYSTEM(omnirobot_FO_t, omnirobot_FO)

auto or_vel_fn = [](prx::system_ptr_t sys_ptr)
{
 //    auto s = std::dynamic_pointer_cast<prx::treaded_vehicle_t>(sys_ptr);

	// prx::space_point_t bk_state = s -> state_space -> make_point();
 //    prx::space_point_t bk_deriv = s -> derivative_space -> make_point();

	// s -> state_space -> copy_to_point(bk_state);
 //    s -> derivative_space -> copy_to_point(bk_deriv);

	// s -> state_space -> at(0) = s -> state_space -> at(1) = s -> state_space -> at(2) = 0;
	// s -> state_space -> at(3) = s -> state_space -> get_bounds()[3].second;
	// s -> state_space -> at(4) = s -> state_space -> get_bounds()[4].second;
            
 //    s -> compute_derivative();
    	double vel = 0;
 //     vel = sqrt(std::pow(s -> state_space -> at(3), 2) + std::pow(s -> state_space -> at(4), 2));

 //    // Copy back the original values
	// s -> state_space -> copy_from_point(bk_state);
	// s -> derivative_space -> copy_from_point(bk_deriv);
	return vel;
};
PRX_REGISTER_VELOCITY_FN(omnirobot_FO, or_vel_fn)
