#include "prx/simulation/plants/types/linear_time_variant.hpp"

namespace prx
{
	ltv_t::ltv_t(std::string _path) : lti_t(_path)
	{
		
	}

    bool ltv_t::linearize(space_point_t xt, space_point_t ut)
    {

    	unsigned ss_dim = ltv_stt_space -> get_dimension(); 
    	unsigned cs_dim = ltv_ctr_space -> get_dimension(); 
    	if (A.size() == 0 || B.size() == 0) 
    	{
    		A.resize(ss_dim, ss_dim);
    		B.resize(ss_dim, cs_dim);
    		x_plus.resize(ss_dim);
    		x_minus.resize(ss_dim);
    		u_plus.resize(cs_dim);
			u_minus.resize(cs_dim);
    	}

		ltv_stt_space -> copy_vector_from_point(x, xt);
		ltv_ctr_space -> copy_from_point(ut);
  		for (int i=0; i < ss_dim; i++) 
  		{
			x_plus  = x;// + simulation_step;
			x_minus = x;// - simulation_step;
			x_plus[i] += simulation_step;
			x_minus[i] -= simulation_step;

			ltv_stt_space -> copy_from_vector(x_plus);
			compute_derivative();
			ltv_stt_space -> copy_to_vector(xd_plus);
			ltv_stt_space -> copy_from_vector(x_minus);
			compute_derivative();
			ltv_stt_space -> copy_to_vector(xd_minus);

    		A.col(i) = ( xd_plus - xd_minus) / ( 2. * simulation_step );

		}
		ltv_stt_space -> copy_from_vector(x);
		ltv_ctr_space -> copy_vector_from_point(u,ut);

  		for (int i=0; i < cs_dim; i++) 
		{
			u_plus  = u;// + simulation_step;
			u_minus = u;// - simulation_step;
			u_plus[i] += simulation_step;
			u_minus[i] -= simulation_step;

			ltv_ctr_space -> copy_from_vector(u_plus);
			compute_derivative();
			ltv_ctr_space -> copy_to_vector(ud_plus);
			ltv_ctr_space -> copy_from_vector(u_minus);
			compute_derivative();
			ltv_ctr_space -> copy_to_vector(ud_minus);

    		B.col(i) = ( ud_plus - ud_minus) / ( 2. * simulation_step );
		}
		return true;
    }

}