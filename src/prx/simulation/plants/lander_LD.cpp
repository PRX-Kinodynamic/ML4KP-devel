#include "prx/simulation/plants/lander_LD.hpp"

namespace prx
{

	lander_LD_t::lander_LD_t(const std::string& path) : plant_t(path)
	{
		m=h=h_dot=0;
		state_memory = {&h,&h_dot,&m};
		state_space = new space_t("EEE",state_memory,"lander_state");
		state_space->set_bounds({0,-2*h_0,M_0},{2*h_0,2*h_0,M_0 + fuel_0});
		// state_space->set_bounds({-3.15,-3.15,-6,-6},{3.15,3.15,6,6});

		u=0;
		control_memory = {&u};
		input_control_space = new space_t("E",control_memory,"Thrust");
		input_control_space->set_bounds({0},{1});

		m_dot=h_dot=h_ddot=0;
		derivative_memory = {&h_dot,&h_ddot,&m_dot};
		derivative_space = new space_t("EEE",derivative_memory,"lander_deriv");


		parameter_memory = {&M_0,&k,&gravity};
		parameter_space = new space_t("EEE", parameter_memory, "lander_params");

		const double length = 20;

		geometries["body"] = std::make_shared<geometry_t>(geometry_type_t::SPHERE);
		geometries["body"] -> initialize_geometry({0.5});
		geometries["body"] -> generate_collision_geometry();
		geometries["body"] -> set_visualization_color("0x00ff00");
		configurations["body"] = std::make_shared<transform_t>();
		configurations["body"] -> setIdentity();

		// set_integrator("rk4");
		set_integrator(integrator_t::kRK4);
		
	}

	lander_LD_t::~lander_LD_t()
	{

	}

	void lander_LD_t::propagate(const double simulation_step)
	{
		integrator -> integrate(simulation_step);

        state_space -> enforce_bounds();
	}

	void lander_LD_t::update_configuration()
	{

		auto body = configurations["body"];
		body->setIdentity();
		Eigen::Matrix3d m;

		body->translation() = (vector_t( 0, 
										 0, 
										0.25+h));
	}

	void lander_LD_t::compute_derivative()
	{
		// h_ddot = (-g * m + alpha ) / m;
		double _u = m <= M_0 ? 0 : u;
		h_ddot = -gravity - _u / m;
		m_dot = -k * _u;
	}

  //   bool lander_LD_t::linearize(space_point_t xt, space_point_t ut, double epsilon)
  //   {
		// return ltv_t::linearize(xt, ut, epsilon);
  //   }

}