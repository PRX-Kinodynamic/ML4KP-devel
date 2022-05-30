#include "prx/simulation/plants/quadrotor_1d.hpp"

namespace prx
{
    quadrotor_1d_t::quadrotor_1d_t(const std::string& path) : plant_t(path)
    {
        z=zdot=0;
        state_memory = {&z, &zdot};
        state_space = new space_t("EE",state_memory,"ZdZ");
        state_space->set_bounds(
            {-10.0,-1.0},
            { 10.0, 1.0}
            );
        
        T=0;
        control_memory = {&T};
        input_control_space = new space_t("E",control_memory,"ddZ");
        input_control_space->set_bounds(
            {-5.0}, {5.0}
            );

        derivative_memory = {&zdot,&zdotdot};
        derivative_space = new space_t("EE",derivative_memory,"dZddZ");

        geometries["body"] = std::make_shared<geometry_t>(geometry_type_t::SPHERE);
        geometries["body"]->initialize_geometry({0.1});
        geometries["body"]->generate_collision_geometry();
        geometries["body"]->set_visualization_color("0x00ff00");
        configurations["body"]= std::make_shared<transform_t>();
        configurations["body"]->setIdentity();

        set_integrator(integrator_t::kEULER);
    }

    quadrotor_1d_t::~quadrotor_1d_t()
    {

    }

    void quadrotor_1d_t::propagate(const double simulation_step)
    {
        integrator -> integrate(simulation_step);
        
    }

    void quadrotor_1d_t::update_configuration()
    {
        auto body = configurations["body"];
        body->setIdentity();
        body->linear() = (quaternion_t(0,0,0,1).toRotationMatrix());
        body->translation() = (vector_t(0,0,z));
    }

    void quadrotor_1d_t::compute_derivative()
    {
        zdotdot = (T / m) - g;
    }
}