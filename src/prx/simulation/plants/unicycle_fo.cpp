#include "prx/simulation/plants/unicycle_fo.hpp"

namespace prx
{
    unicycle_fo_t::unicycle_fo_t(const std::string& path) : plant_t(path)
    {
        x=y=theta=0;
        state_memory = {&x,&y,&theta};
		state_space = new space_t("EER",state_memory,"TreadedFOState");
		//**state_space->set_bounds({0,0,-3.15},{5,5,3.15});
		state_space->set_bounds({0,0,-3.15},{3,1.2,3.15});
		
		//IMRCLab state_space->set_bounds({0,0,-3.15},{6,6,3.15});
		
        v=0;
        control_memory = {&v,&dtheta};
        input_control_space = new space_t("EE",control_memory,"TreadedFOControl");
        // v0
        // input_control_space->set_bounds({-0.5,-0.5},{0.5,0.5});
        // v1
        // input_control_space->set_bounds({0.2,-0.5},{0.5,0.5});
        // v2
        input_control_space->set_bounds({-0.5,-0.25},{0.5,0.5});


        dx=dy=dtheta=0;
        derivative_memory = {&dx,&dy,&dtheta};
        derivative_space = new space_t("EEE",derivative_memory,"ThetaFODerivative");

        geometries["body"] = std::make_shared<geometry_t>(geometry_type_t::BOX);
		geometries["body"]->initialize_geometry({.5,.25, 1.0});
		geometries["body"]->generate_collision_geometry();
		geometries["body"]->set_visualization_color("0xff00ff");
		configurations["body"]= std::make_shared<transform_t>();
		configurations["body"]->setIdentity();

		set_integrator(integrator_t::kEULER);
    }

    unicycle_fo_t::~unicycle_fo_t() {}

    void unicycle_fo_t::set_state_space_bounds(const std::vector<double>& lower,const std::vector<double>& upper)
	{
		state_space->set_bounds(lower, upper);
	}
	
    void unicycle_fo_t::propagate(const double simulation_step)
    {
        integrator -> integrate(simulation_step);
    }

    // void unicycle_fo_t::set_state_space_bounds(const std::vector<double>& lower,const std::vector<double>& upper)
    // {
    //     for (int i = 0; i < std::min(lower.size(), lower_bound.size()); ++i)
    //     {
    //         lower_bound[i] = lower[i];
    //     }
    //     for (int i = 0; i < std::min(upper.size(), upper_bound.size()); ++i)
    //     {
    //         upper_bound[i] = upper[i];
    //     }

    //     state_space->set_bounds(lower_bound, upper_bound);
    // }

    void unicycle_fo_t::update_configuration()
    {
        auto body = configurations["body"];
		body->setIdentity();
		body->linear() = (quaternion_t(cos(theta/2),0,0,sin(theta/2)).toRotationMatrix());
		body->translation() = (vector_t(x,y,0));
    }

    void unicycle_fo_t::compute_derivative()
    {
        dx = v*cos(theta);
        dy = v*sin(theta);
    }
}
