#include "prx/simulation/plants/trailer_car.hpp"

namespace prx
{
    trailer_car_t::trailer_car_t(const std::string& path) : plant_t(path)
    {
        x=y=theta0=theta1=0;
        state_memory={&x,&y,&theta0,&theta1};
        state_space = new space_t("EERR", state_memory,"TrailerCarState");
        state_space->set_bounds({-11,-11,-3.15,-3.15},{11,11,3.15,3.15});

        v=phi=0;
        control_memory={&v,&phi};
        input_control_space = new space_t("ER", control_memory,"TrailerCarControl");
        input_control_space->set_bounds({-0.1,-PRX_PI/3},{0.5,PRX_PI/3});

        dx=dy=dtheta0=dtheta1=0;
        derivative_memory={&dx,&dy,&dtheta0,&dtheta1};
        derivative_space = new space_t("EEEE", derivative_memory,"TrailerCarDerivative");

        geometries["body1"] = std::make_shared<geometry_t>(geometry_type_t::BOX);
		geometries["body1"]->initialize_geometry({.5,.25, 1.0});
		geometries["body1"]->generate_collision_geometry();
		geometries["body1"]->set_visualization_color("0xff00ff");
		configurations["body1"]= std::make_shared<transform_t>();
		configurations["body1"]->setIdentity();

        geometries["body2"] = std::make_shared<geometry_t>(geometry_type_t::BOX);
		geometries["body2"]->initialize_geometry({.5,.25, 1.0});
		geometries["body2"]->generate_collision_geometry();
		geometries["body2"]->set_visualization_color("0xff00ff");
		configurations["body2"]= std::make_shared<transform_t>();
		configurations["body2"]->setIdentity();

        set_integrator(integrator_t::kEULER);
    }

    trailer_car_t::~trailer_car_t() {}

    void trailer_car_t::propagate(const double simulation_step)
    {
        integrator -> integrate(simulation_step);
    }

    void trailer_car_t::set_state_space_bounds(const std::vector<double>& lower, const std::vector<double>& upper)
    {
        state_space->set_bounds(lower,upper);
    }

    void trailer_car_t::update_configuration()
    {
        prx_assert(false,"Not implemented!");
        // configurations["body1"]->setIdentity();
        // configurations["body1"]->setOrigin(tf::Vector3(x,y,0));
        // configurations["body1"]->setRotation(tf::createQuaternionFromRPY(0,0,theta0));

        // configurations["body2"]->setIdentity();
        // configurations["body2"]->setOrigin(tf::Vector3(x+L*cos(theta0),y+L*sin(theta0),0));
        // configurations["body2"]->setRotation(tf::createQuaternionFromRPY(0,0,theta1));
    }

    void trailer_car_t::compute_derivative()
    {
        dx = v*cos(theta0);
        dy = v*sin(theta0);
        dtheta0 = v*std::tan(phi)/L;
        dtheta1 = v*std::sin(theta0-theta1)/d1;
    }
}