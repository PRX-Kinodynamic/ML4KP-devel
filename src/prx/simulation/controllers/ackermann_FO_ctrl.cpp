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
		const double x = (*aux_pt)[0];
		const double y = (*aux_pt)[1];
		const double theta = (*aux_pt)[2];
		const double L = 1;

		double x_diff = (*goal)[0] - x;
		double y_diff = (*goal)[1] - y;

		rho = std::sqrt(std::pow(x_diff, 2) + std::pow(y_diff, 2));
		alpha = std::atan(y_diff/x_diff) - theta;
        
		beta = -theta - alpha + (*goal)[2];
		V = k_rho * rho;
		double omega = k_alpha * alpha + k_beta * beta;
		if (rho == 0)
		{
			gamma = 0;
		}
		else if ( -M_PI/2. < alpha && alpha < M_PI / 2.  )
		{
			gamma = std::atan(omega * L / V);
		}
		else
		{
			gamma = std::atan(omega * L / V);
			// V = -V;
		}
		// printf("x_diff: %.4f, y_diff: %.4f, rho: %.4f, V: %.4f\n", x_diff, y_diff, rho, V);

  //       beta = -std::atan2(-y_diff, -x_diff);
  //       alpha = -theta - beta ;
  //       if ( aFO -> get_control_space() -> get_lower_bound(1) < 0.0 && ( (alpha > M_PI / 2.0) || (alpha < -M_PI / 2.0) ) )
  //       {
  //          	direccion = -1;
  //       }
  //       else
  //       {
  //          	direccion = 1;
  //       }
  //       if (direccion == -1)
  //       {
  //       	// Going backwards
		// 	beta = -std::atan2(y_diff, x_diff) ;
  //       }
  //       else
  //       {
  //       	// Going forwards
  //       	beta = -std::atan2(-y_diff, -x_diff);
  //       }

  //       if ( alpha > M_PI / 2. )
  //       {
  //       	alpha = M_PI/2;
  //       }
    	
  //   	if ( alpha < -M_PI/2. )
  //   	{
  //       	alpha = -M_PI/2;
  //   	}

		// V = direccion * k_rho * rho;
		// if (V == 0)
		// {
		// 	gamma = 0;
		// }
		// else
		// {
		// 	beta += (*goal)[2];
		// 	double omega = k_alpha * alpha + k_beta * beta;
		// 	gamma = std::atan(direccion * omega / std::abs(V));
		// }
		(*ctrl_pt)[0] = gamma;
		(*ctrl_pt)[1] = V;
		// std::cout << "ctrl_pt: " << ctrl_pt << std::endl;
		aFO -> get_control_space() -> copy_from_point(ctrl_pt);
		aFO -> get_control_space() -> enforce_bounds();
		// std::cout << "aFO: " << plant << std::endl; 
	}

}