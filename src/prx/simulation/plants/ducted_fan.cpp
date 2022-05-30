#include "prx/simulation/plants/ducted_fan.hpp"

namespace prx
{
    ducted_fan_t::ducted_fan_t(const std::string& path) : plant_t(path)
    {
        x=y=theta=dx=dy=dtheta=0;
        state_memory = {&x,&y,&theta,&dx,&dy,&dtheta};
        state_space = new space_t("EEREEE",state_memory,"DuctedFanState");
        state_space -> set_bounds(lower_bound, upper_bound);

        u1=u2=0;
        control_memory = {&u1,&u2};
        input_control_space = new space_t("EE",control_memory,"DuctedFanControl");
        input_control_space -> set_bounds({-.1,-.1},{.1,.1});

        ddx=ddy=ddtheta=0;
        derivative_memory = {&dx,&dy,&dtheta,&ddx,&ddy,&ddtheta};
        derivative_space = new space_t("EEEEEE",derivative_memory,"DuctedFanDerivative");

        geometries["body"] = std::make_shared<geometry_t>(geometry_type_t::BOX);
		geometries["body"]->initialize_geometry({.5,.25, 1.0});
		geometries["body"]->generate_collision_geometry();
		geometries["body"]->set_visualization_color("0xff00ff");
		configurations["body"]= std::make_shared<transform_t>();
		configurations["body"]->setIdentity();

		set_integrator(integrator_t::kEULER);
    }

    ducted_fan_t::~ducted_fan_t() {}

    void ducted_fan_t::propagate(const double simulation_step)
    {
        integrator -> integrate(simulation_step);
    }

    void ducted_fan_t::update_configuration()
    {
        auto body = configurations["body"];
		body->setIdentity();
		body->linear() = (quaternion_t(cos(theta/2),0,0,sin(theta/2)).toRotationMatrix());
		body->translation() = (vector_t(x,y,0));
    }

    void ducted_fan_t::compute_derivative()
    {
        ddx = (-d*dx + u1*std::cos(theta) - u2*std::sin(theta))/m;
        ddy = (-d*dy + u1*std::sin(theta) - u2*std::cos(theta) -m*g)/m;
        ddtheta = r*u1/I;
    }
}