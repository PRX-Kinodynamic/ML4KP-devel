
#include "prx/simulation/plants/pendulum.hpp"

namespace prx
{

	pendulum_t::pendulum_t(const std::string& path) : plant_t(path), lti_t(state_space, input_control_space)
	{
		_theta1=_theta1dot=0;
		state_memory = {&_theta1,&_theta1dot};
		state_space = new space_t("RE",state_memory,"pendulum_state");
		state_space->set_bounds({0,-6},{2.0*M_PI,6});
		// state_space->set_bounds({-3.15,-3.15,-6,-6},{3.15,3.15,6,6});

		_tau=0;
		control_memory = {&_tau};
		input_control_space = new space_t("E",control_memory,"Torque");
		input_control_space->set_bounds({-7},{7});

		_theta1dotdot=0;
		derivative_memory = {&_theta1dot,&_theta1dotdot};
		derivative_space = new space_t("EE",derivative_memory,"pendulum_deriv");

		const double length = 20;

		geometries["rod1"] = std::make_shared<geometry_t>(geometry_type_t::BOX);
		geometries["rod1"]->initialize_geometry({length,1,1});
		geometries["rod1"]->generate_collision_geometry();
		geometries["rod1"]->set_visualization_color("0x00ff00");
		configurations["rod1"]= std::make_shared<transform_t>();
		configurations["rod1"]->setIdentity();

		geometries["ball"] = std::make_shared<geometry_t>(geometry_type_t::SPHERE);
		geometries["ball"]->initialize_geometry({1.5});
		geometries["ball"]->generate_collision_geometry();
		geometries["ball"]->set_visualization_color("0x0000ff");
		configurations["ball"]= std::make_shared<transform_t>();
		configurations["ball"]->setIdentity();

		// set_integrator("rk4");
		set_integrator(integrator_t::kRK4);

	}

	pendulum_t::~pendulum_t()
	{

	}

	void pendulum_t::propagate(const double simulation_step)
	{
		integrator -> integrate(simulation_step);

        _theta1 = norm_angle_pi(_theta1, 0, 2*M_PI);
        state_space -> enforce_bounds();
	}

	void pendulum_t::update_configuration()
	{

		const double length = 20;
		auto body = configurations["rod1"];
		body->setIdentity();
		body->linear() = (quaternion_t(cos((_theta1 - PRX_PI / 2) / 2.0), 0, 0, sin((_theta1 - PRX_PI / 2) / 2.0)).toRotationMatrix());
		body->translation() = (vector_t((length / 2.0) * cos(_theta1 - PRX_PI / 2), 
										(length / 2.0) * sin(_theta1 - PRX_PI / 2), 
										1.5));

		body = configurations["ball"];
		body->setIdentity();
		body->translation() = (vector_t((length) * cos(_theta1 - PRX_PI / 2),
                                        (length) * sin(_theta1 - PRX_PI / 2),
                                        1.5));
	}

	void pendulum_t::compute_derivative()
	{
		inertia = mass * length * length;


        _theta1dotdot = gravity / length * std::sin(_theta1) + _tau / inertia;

        // if (friction > 0)
        // {
            _theta1dotdot -= friction / inertia * _theta1dot;
        // }

	}
}