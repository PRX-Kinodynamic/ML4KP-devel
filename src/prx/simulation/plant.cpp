
#include "prx/simulation/plant.hpp"
#include "prx/utilities/defs.hpp"


namespace prx
{

	plant_t::plant_t(const std::string& path) : system_t(path),movable_object_t(path)
	{
		derivative_space=nullptr;

		collision_list.clear();
		set_integrator(integrator_t::kEULER);
	}

	plant_t::~plant_t()
	{
		if(derivative_space!=nullptr)
		{
			delete derivative_space;
		}
	}

	plant_t::plant_t(const std::shared_ptr<plant_t>& _plant) 
	: system_t(_plant),
	  movable_object_t(_plant -> get_pathname())
	{
		// public stuff
		derivative_space = _plant -> derivative_space;
		derivative_memory = _plant -> derivative_memory;

		// protected stuff
		integrator = _plant -> integrator;
		collision_list = _plant -> collision_list;

		// private stuff:
		derivative_state = _plant -> derivative_state;
	}

	void plant_t::set_integrator(integrator_t::integrators integrator_choice)
	{
		double initial_integration_step = simulation_step;
		//printf("simulation_step: %.6f\n", simulation_step);
		std::function<void ()> deriv_f = [this](){this -> compute_derivative();};
		if (integrator_choice == integrator_t::kEULER) 
			integrator = std::make_shared<euler_t>(state_space, derivative_space, deriv_f, initial_integration_step);
		else if (integrator_choice == integrator_t::kRK4) 
			integrator = std::make_shared<runge_kutta4_t>(state_space, derivative_space, deriv_f, initial_integration_step);
		else if (integrator_choice == integrator_t::kDOPRI5) 
			integrator = std::make_shared<dopri5_t>(state_space, derivative_space, deriv_f, initial_integration_step);
		else printf("Invalid integrator choice. Available integrators are: euler, rk4 and dopri5\n");

	}	
    
	void plant_t::add_system(system_ptr_t& ptr)
	{
		prx_throw("Trying to add a system ("<<ptr->get_pathname()<<") into a plant ("<<pathname<<")");
	}

	void plant_t::propagate(const double simulation_step)
	{
		integrator -> integrate(simulation_step);
		//euler_integration(simulation_step);
	}

	void plant_t::compute_control()
	{
		// No controls for the regular abstract plant
	}

	void plant_t::compute_stopping_maneuver(space_point_t start_state, std::vector<double>& times, std::vector<double>& ctrls)
	{
		//the control should have already been set in the vehicle space
	}
	
	void plant_t::set_state_space_bounds(const std::vector<double>& lower,const std::vector<double>& upper)
	{
		state_space->set_bounds(lower, upper);
	}

	// void plant_t::gradient(space_point_t x, space_point_t u, bool xu, Eigen::VectorXd out )
	// {
	// 	const double epsilon = std::sqrt(simulation_step);
 //    	unsigned ss_dim = get_state_space() -> get_dimension(); 
 //    	unsigned cs_dim = get_control_space() -> get_dimension(); 

 //    	unsigned dd_dim = xu ? ss_dim : cs_dim;

 //    	out.resize(ss_dim);

 //    	plus.resize(dd_dim);
 //    	minus.resize(dd_dim);

 //    	v.resize(dd_dim);

	// 	get_state_space() -> copy_vector_from_point(xv, x);
	// 	get_state_space() -> copy_vector_from_point(uv, u);


	// 	for (int i=0; i < dd_dim; i++) 
 //  		{
	// 		plus  = v;
	// 		minus = v;
	// 		plus(i) += epsilon;
	// 		minus(i) -= epsilon;

	// 		if (xu) compute_derivative(plus, uv);
	// 		else    compute_derivative(xv, plus);
	// 		propagate(epsilon);
	// 		get_derivative_space() -> copy_to_vector(d_plus);


	// 		if (xu) compute_derivative(minus, uv);
	// 		else    compute_derivative(xv, minus);
	// 		propagate(epsilon);
	// 		get_derivative_space() -> copy_to_vector(d_minus);

 //    		out.col(i) = ( d_plus - d_minus) / ( 2. * epsilon );

	// 	}
	// }

}
