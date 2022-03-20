
#include "prx/simulation/plants/pendulum.hpp"

namespace prx
{

	pendulum_t::pendulum_t(const std::string& path) : ltv_t(path)
	{
		_theta1=_theta1dot=0;
		state_memory = {&_theta1,&_theta1dot};
		state_space = new space_t("RE",state_memory,"pendulum_state");
		state_space->set_bounds({-PRX_PI,-2*PRX_PI},{PRX_PI,2*PRX_PI});
		// state_space->set_bounds({-3.15,-3.15,-6,-6},{3.15,3.15,6,6});

		_tau=0;
		control_memory = {&_tau};
		input_control_space = new space_t("E",control_memory,"Torque");
		input_control_space->set_bounds({-0.6371781908344007},{0.6371781908344007});

		_theta1dotdot=0;
		derivative_memory = {&_theta1dot,&_theta1dotdot};
		derivative_space = new space_t("EE",derivative_memory,"pendulum_deriv");


		parameter_memory = {&length,&friction,&mass,&normalize};
		parameter_space = new space_t("EEED", parameter_memory, "pendulum_params");
		
	}

	pendulum_t::~pendulum_t()
	{

	}

	void pendulum_t::propagate(const double simulation_step)
	{

	}

	void pendulum_t::update_configuration()
	{
        const double theta1 = _theta1 - M_PI / 2.0;

		const double length = 20;
		auto body = configurations["rod1"];
		body->setIdentity();
		Eigen::Matrix3d m;
		m =   Eigen::AngleAxisd(-theta1, Eigen::Vector3d::UnitZ())
		    * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
		    * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
		body->linear() = (m);
		body->translation() = (vector_t((length / 2.0) * cos(theta1), 
										-(length / 2.0) * sin(theta1), 
										1.5));

		body = configurations["ball"];
		body->setIdentity();
		body->translation() = (vector_t((length) * cos(theta1),
                                        -(length) * sin(theta1),
                                        1.5));
	}

	void pendulum_t::compute_derivative()
	{

	}


}