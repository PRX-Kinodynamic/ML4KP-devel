#include "prx/simulation/controllers/ackermann_FO_ctrl.hpp"


namespace prx
{

	void ackermann_FO_ctrl_t::set_gains(double _k_rho, double _k_alpha, double _k_beta)
	{
		k_rho = _k_rho;
		k_alpha = _k_alpha;
		k_beta = _k_beta;
	}

	void ackermann_FO_ctrl_t::compute_controls()
	{	
		aFO -> get_state_space() -> copy_to_point(aux_pt);
		const double L = 1;
		const double x = (*aux_pt)[0];
		const double y = (*aux_pt)[1];
		const double theta = (*aux_pt)[2];

		// double x_diff = x - (*goal)[0];
		// double y_diff = y - (*goal)[1];
		double x_diff = x - (*goal)[0];
		double y_diff = y - (*goal)[1];

		movepoint_sfunc(x_diff, y_diff, theta);

		// printf("vals: (%.2f, %.2f, %.2f, %.0f)\n", rho, alpha, beta, direccion);
		beta += theta;

		V = direccion * k_rho * rho;
		auto omega = k_alpha * alpha + k_beta * beta;

		auto u = direccion * omega / std::abs(V);

		gamma = std::atan(u);

		(*ctrl_pt)[0] = gamma;
		(*ctrl_pt)[1] = V;

		aFO -> get_control_space() -> copy_from_point(ctrl_pt);
		aFO -> get_control_space() -> enforce_bounds();
	}

	void ackermann_FO_ctrl_t::movepoint_sfunc(double x, double y, double theta)
	{
		// aFO -> get_state_space() -> copy_to_point(aux_pt);
		// const double x = (*aux_pt)[0];
		// const double y = (*aux_pt)[1];
		// const double theta = (*aux_pt)[2];

		rho = sqrt(x*x + y*y);

		// std::cout << "direccion: " << direccion << std::endl;
    	if ( direccion == 0 )
    	{
    		// printf("direccion is ZERO!!");
        	beta = -std::atan2(-y, -x);
        	alpha = -theta - beta;
        	// fprintf('alpha %f, beta %f\n', alpha, beta);
        	// % first time in simulation, choose the direction of travel
        	if (  aFO -> get_control_space() -> get_lower_bound(1) < 0.0 && ( (alpha > PRX_PI / 2.0) || (alpha < -PRX_PI / 2.0) ) )
        	{
        	    // printf("going backwards\n");
        	    direccion = -1;
        	}
        	else
        	{   
        		// printf("going forwards\n");
        	    direccion = 1;
        	}
    	}
    	else if ( direccion == -1 )
    	{
        	beta = -std::atan2(y, x);
        	alpha = -theta - beta;
    	}
    	else
    	{
        	beta = -std::atan2(-y, -x);
        	alpha = -theta - beta;
    	}
    	if ( alpha > PRX_PI / 2.0 )
        	alpha = PRX_PI / 2.0;
    	
    	if ( alpha < -PRX_PI / 2.0 )
	        alpha = -PRX_PI / 2.0;
    	
	}
}