#include "prx/simulation/plants/unicycle.hpp"

namespace prx
{
    unicycle_t::unicycle_t(const std::string& path) : plant_t(path)
    {
        x=y=theta=v=omega=0;
        state_memory = {&x,&y,&theta,&v,&omega};
        state_space = new space_t("EEREE",state_memory,"UnicycleState");
        state_space->set_bounds(lower_bound,upper_bound);

        ul=ur=0;
        control_memory = {&ul,&ur};
        input_control_space = new space_t("EE",control_memory,"UnicycleControl");
        input_control_space->set_bounds(ctrl_lower_bound,ctrl_upper_bound);

        dx=dy=0;
        derivative_memory = {&dx,&dy,&omega,&ul,&ur};
        derivative_space = new space_t("EEEEE",derivative_memory,"UnicycleDerivative");

        geometries["body"] = std::make_shared<geometry_t>(geometry_type_t::BOX);
		geometries["body"]->initialize_geometry({.508,.430,.25});
		geometries["body"]->generate_collision_geometry();
		geometries["body"]->set_visualization_color("0xff00ff");
		configurations["body"]= std::make_shared<transform_t>();
		configurations["body"]->setIdentity();

        set_integrator(integrator_t::kEULER);
    }

    unicycle_t::~unicycle_t()
    {

    }

    void unicycle_t::compute_stopping_maneuver(space_point_t start_state, double& time)
    {
        std::vector<double> desired_acceleration = {-start_state -> at(3)/time, -start_state -> at(4)/time};
        double max_time_required = time;
        double multiplier = 1.0/simulation_step;
        
        for (int i = 0; i < input_control_space->get_dimension(); i++)
        {
            if (desired_acceleration[i] < ctrl_lower_bound[0])
            {
                desired_acceleration[i] = ctrl_lower_bound[0];
            }
            else if (desired_acceleration[i] > ctrl_upper_bound[0])
            {
                desired_acceleration[i] = ctrl_upper_bound[0];
            }
            max_time_required = simulation_step * std::ceil(multiplier*std::max(max_time_required, -start_state -> at(3+i)/desired_acceleration[i]));
        }

        if (max_time_required > time)
        {
            time = max_time_required;
        }
    }
    
    void unicycle_t::propagate(const double simulation_step)
    {
        integrator -> integrate(simulation_step);
    }

    void unicycle_t::update_configuration()
	{
		auto body = configurations["body"];
		body->setIdentity();
		body->linear() = (quaternion_t(cos(theta/2),0,0,sin(theta/2)).toRotationMatrix());
		body->translation() = (vector_t(x,y,0));
	}

    void unicycle_t::compute_derivative()
    {
        dx = v*cos(theta);
        dy = v*sin(theta);
    }
}