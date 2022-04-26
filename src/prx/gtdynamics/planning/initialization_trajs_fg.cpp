#include "prx/gtdynamics/planning/initialization_trajs_fg.hpp"


namespace prx
{

	gtsam::Values initialization_trajs_fg_t::zero_state(const system_ptr_t _sys_ptr, const int t, const int num_steps, const double sigma)
	{
		gtsam::Values values;
		auto ss = _sys_ptr -> get_state_space();
		auto cs = _sys_ptr -> get_control_space();

  		auto x_dim = ss -> get_dimension();
  		auto u_dim = cs -> get_dimension();

		auto sampler_noise_model = 
			gtsam::noiseModel::Isotropic::Sigma(x_dim, sigma);
		auto sampler_noise_ctrl = 
			gtsam::noiseModel::Isotropic::Sigma(u_dim, sigma);

  		gtsam::Sampler x_sampler(sampler_noise_model);
  		gtsam::Sampler u_sampler(sampler_noise_ctrl);

  		auto xi_sy = prx_symbol_t::state_symbol(t);
  		values.insert(xi_sy, x_sampler.sample());

  		if (t < num_steps)
  		{
  			auto ui_sy = prx_symbol_t::control_symbol(t);
  			values.insert(ui_sy, u_sampler.sample());
  		}

		return values;
	}

	gtsam::Values initialization_trajs_fg_t::state_from_space(const space_t* space, const int t, const bool state_or_ctrl, const double sigma)
	{
		gtsam::Values values;

  		auto dim = space -> get_dimension();

		auto sampler_noise = 
			gtsam::noiseModel::Isotropic::Sigma(dim, sigma);

  		gtsam::Sampler sampler(sampler_noise);

  		prx_symbol_t st = state_or_ctrl ? prx_symbol_t::state_symbol(t) : prx_symbol_t::control_symbol(t);
  			
  		Eigen::VectorXd v(dim);
  		space -> copy_to_vector(v);
  		v += sampler.sample();
  		values.insert(st, v);

		return values;
	}

	gtsam::Values initialization_trajs_fg_t::constant_trajectory(const system_ptr_t _sys_ptr, const space_point_t start_state, const space_point_t x, const space_point_t u, const int num_steps, const double sigma)
	{
		gtsam::Values values;
		auto ss = _sys_ptr -> get_state_space();
		auto cs = _sys_ptr -> get_control_space();

		cs -> copy_from_point(u);

  		for (int t = 0; t <= num_steps; t++)
	  	{
	  		if (t == 0)
	  		{ 
	  			// std::cout << "start_state: " << start_state << std::endl;
	  			ss -> copy_from_point(start_state);
	   			values.insert(state_from_space(ss, t, true, 0.0));
	  		}
	  		else
	  		{
	  			// std::cout << "x: " << start_state << std::endl;
				ss -> copy_from_point(x);
	   			values.insert(state_from_space(ss, t, true, sigma));
	  		}
  			if (t < num_steps)
			{
				// cs -> sample(u);
				cs -> copy_from_point(u);
				values.insert(state_from_space(cs, t, false, sigma));
			}
  		}
  		return values;
	}

	gtsam::Values initialization_trajs_fg_t::init_time_factors(const int num_steps, const double sigma)
	{
		gtsam::Values values;

		auto sampler_noise = 
			gtsam::noiseModel::Isotropic::Sigma(1, sigma);

  		gtsam::Sampler sampler(sampler_noise);

  		for (int t = 0; t < num_steps; ++t)
  		{
  			auto ti = prx_symbol_t::time_symbol(t);
  			Eigen::VectorXd v(1);
  			v[0] = std::fabs( simulation_step + sampler.sample()[0] );
	  		// v += sampler.sample();
	  		// v[0] = std::fabs(v[0]);
	  		values.insert(ti, v);
  		}
  			
		return values;
	}

	gtsam::Values initialization_trajs_fg_t::zeros_trajectory(const system_ptr_t _sys_ptr, const int num_steps, const double sigma)
	{
		gtsam::Values z_values;
  		for (int t = 0; t <= num_steps; t++)
	  	{
	   		z_values.insert(zero_state(_sys_ptr, t, num_steps, sigma));
  		}
  		return z_values;
	}

	gtsam::Values initialization_trajs_fg_t::init_from_plan(
		std::shared_ptr<system_group_t> sg, const space_point_t start_state, const plan_t& plan, int& total_steps, double sigma)
	{
		gtsam::Values values;

		auto ss = sg -> get_state_space();
		auto cs = sg -> get_control_space();
	
		auto x_dim = ss -> get_dimension();
		auto u_dim = cs -> get_dimension();

		auto aux_pt = ss -> make_point();

		auto x_sampler_noise = gtsam::noiseModel::Isotropic::Sigma(x_dim, sigma);
		auto u_sampler_noise = gtsam::noiseModel::Isotropic::Sigma(u_dim, sigma);

  		gtsam::Sampler x_sampler(x_sampler_noise);
  		gtsam::Sampler u_sampler(u_sampler_noise);

		total_steps = 0; 
		auto xi_sy = prx_symbol_t::state_symbol(total_steps);
		Eigen::VectorXd vs = gtsam::Vector::Zero(x_dim);
		ss -> copy_vector_from_point(vs, start_state);
		// vs += x_sampler.sample();
		values.insert(xi_sy, vs );
		
		
		ss -> copy_from_point(start_state);
		for (auto step : plan)
		{
			for (double i = 0; i < step.duration; i += simulation_step)
			{
				auto ui_sy = prx_symbol_t::control_symbol(total_steps);
				// std::cout << std::string(ui_sy) << std::endl;
				Eigen::VectorXd vu = gtsam::Vector::Zero(u_dim);
				cs -> copy_vector_from_point(vu, step.control);
				vu += u_sampler.sample();
				values.insert(ui_sy, vu );

				total_steps++; 
				sg -> propagate_once(propagate_step::MIDDLE_STEP, step.control);
				auto xi_sy = prx_symbol_t::state_symbol(total_steps);
				Eigen::VectorXd vs = gtsam::Vector::Zero(x_dim);
				ss -> copy_to_vector(vs);
				// ss -> copy_vector_from_point(vs, aux_pt);
				vs += x_sampler.sample();
				values.insert(xi_sy, vs );
				
				// t++;
			}
		}
		std::cout << "Last t: " << total_steps << std::endl;

		// t=0;

		// for (auto state : traj)
		// {
		// 	auto xi_sy = prx_symbol_t::state_symbol(t);
		// 	// std::cout << std::string(xi_sy) << std::endl;
		// 	Eigen::VectorXd vs = gtsam::Vector::Zero(x_dim);
		// 	ss -> copy_vector_from_point(vs, state);
		// 	values.insert(xi_sy, vs);

		// 	t++;
		// }
		// std::cout << "Last t: " << t << std::endl;
		return values;
	}

	gtsam::Values initialization_trajs_fg_t::init_from_traj(const system_ptr_t _sys_ptr, const trajectory_t& traj, const plan_t& plan)
	{
		gtsam::Values values;

		auto ss = _sys_ptr -> get_state_space();
		auto cs = _sys_ptr -> get_control_space();
	
		auto x_dim = ss -> get_dimension();
		auto u_dim = cs -> get_dimension();
	
		int t = 0;
		for (auto state : traj)
		{
			auto xi_sy = prx_symbol_t::state_symbol(t);
			// std::cout << std::string(xi_sy) << std::endl;
			Eigen::VectorXd vs = gtsam::Vector::Zero(x_dim);
			ss -> copy_vector_from_point(vs, state);
			values.insert(xi_sy, vs);

			t++;
		}

		t = 0; 
		for (auto step : plan)
		{
			for (double i = 0; i < step.duration; i += simulation_step)
			{
				auto ui_sy = prx_symbol_t::control_symbol(t);
				// std::cout << std::string(ui_sy) << std::endl;
				Eigen::VectorXd vu = gtsam::Vector::Zero(u_dim);
				cs -> copy_vector_from_point(vu, step.control);
				values.insert(ui_sy, vu);
				t++;
			}
		}
		std::cout << "Last t: " << t << std::endl;
		return values;
	}

	gtsam::Values initialization_trajs_fg_t::state_to_value(const system_ptr_t _sys_ptr, const space_point_t pt, const int t)
	{
		gtsam::Values values;
		auto ss = _sys_ptr -> get_state_space();
		auto cs = _sys_ptr -> get_control_space();

		auto x_dim = ss -> get_dimension();
		auto u_dim = cs -> get_dimension();

		auto xi_sy = prx_symbol_t::state_symbol(t);
		Eigen::VectorXd vs(x_dim);
		ss -> copy_vector_from_point(vs, pt);
		values.insert(xi_sy, vs);

		return values;
	}

	gtsam::Values initialization_trajs_fg_t::control_to_value(const system_ptr_t _sys_ptr, const space_point_t pt, const int t)
	{
		gtsam::Values values;
		auto cs = _sys_ptr -> get_control_space();
		auto u_dim = cs -> get_dimension();

		auto ui_sy = prx_symbol_t::control_symbol(t);
		Eigen::VectorXd vu(u_dim);
		cs -> copy_vector_from_point(vu, pt);
		values.insert(ui_sy, vu);

		return values;
	}

	gtsam::Values initialization_trajs_fg_t::linear_trajectory(const system_ptr_t _sys_ptr, 
		const space_point_t& start, const space_point_t& goal, 
		const int num_steps)
	{
		gtsam::Values values;
		auto ss = _sys_ptr -> get_state_space();
		auto cs = _sys_ptr -> get_control_space();

		auto x_dim = ss -> get_dimension();
		auto u_dim = cs -> get_dimension();

		auto interpolated = ss -> make_point();
		auto zero_ctrl = cs -> make_point();

		// for (int i = 0; i < u_dim; ++i)
		// {
		// 	(*zero_ctrl)[i] = 0;
		// }

		for (int t = 0; t <= num_steps; t++)
		{
			double t_i = t / static_cast<double>(num_steps);
			ss -> interpolate(start, goal, t_i, interpolated);
			// std::cout << "interpolated: " << interpolated << std::endl;
			values.insert(state_to_value(_sys_ptr, interpolated, t));
			if (t < num_steps)
			{
				cs -> sample(zero_ctrl);
				values.insert(control_to_value(_sys_ptr, zero_ctrl, t));

			}
		}
		return values;
	}


}