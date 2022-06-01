#include "prx/simulation/plants/mountain_car.hpp"

namespace prx
{
    mountain_car_t::mountain_car_t(const std::string& path) : plant_t(path)
    {
        x = xdot = 0;
        state_memory = {&x, &xdot};
        state_space = new space_t("EE",state_memory,"XXdot");
        state_space->set_bounds(
            {-1.2,-1.5},
            { 0.6, 1.5}
            );
        
        u=0;
        control_memory = {&u};
        input_control_space = new space_t("E",control_memory,"u");
        input_control_space->set_bounds(
            {-.2}, {.2}
            );
        
        derivative_memory = {&xdot,&xdotdot};
        derivative_space = new space_t("EE",derivative_memory,"XdotXdotdot");

        set_integrator(integrator_t::kEULER);
    }

    mountain_car_t::~mountain_car_t()
    {

    }

    void mountain_car_t::propagate(const double simulation_step)
    {
        integrator -> integrate(simulation_step);
    }

    void mountain_car_t::update_configuration()
    {

    }

    void mountain_car_t::compute_derivative()
    {
    	oldxdot = xdot;
        xdot = m * g * std::cos(3 * x) + (u / m) -k * oldxdot;
    }
}
