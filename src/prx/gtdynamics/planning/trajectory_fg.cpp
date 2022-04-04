#include "prx/gtdynamics/planning/trajectory_fg.hpp"

namespace prx
{
	trajectory_fg_t::trajectory_fg_t(system_ptr_t _sys_ptr)
	{
		prx_assert(_sys_ptr != nullptr, "trajectory_fg_t cannot handle nullptr systems!");
		plant_ptr = std::dynamic_pointer_cast<plant_t>(_sys_ptr);
		prx_assert(plant_ptr != nullptr, "A type plant_t is needed!");
		// sys_ptr = _sys_ptr;
		dynamics_model = //nullptr;
			gtsam::noiseModel::Isotropic::Sigma(
			plant_ptr -> get_state_space() -> get_dimension(),
			1e-3);
		control_model = //nullptr;
			gtsam::noiseModel::Isotropic::Sigma(
			plant_ptr -> get_control_space() -> get_dimension(),
			1e-4);    // Dynamics constraints.
	}

	gtsam::NonlinearFactorGraph trajectory_fg_t::limits_factors(const int t, const int num_steps) const 
    {
  		gtsam::NonlinearFactorGraph graph;

  		auto ss = plant_ptr -> get_state_space();
  		auto cs = plant_ptr -> get_control_space();

  		auto sx_t = prx_symbol_t::state_symbol(t);
  		auto su_t = prx_symbol_t::control_symbol(t);

  		graph.add(
  			space_limit_factor_t(sx_t, dynamics_model, ss)
  			);
  	  	
  	  	if (t < num_steps) 
  	  	{
  			graph.add(
	  			space_limit_factor_t(su_t, control_model, cs)
	  			);
  	  	}

  		return graph;
	}
	
	gtsam::NonlinearFactorGraph trajectory_fg_t::collocationFactors(const int t)
	{

  		// for (auto x : sys_ptr -> get_state_space())
  		gtsam::NonlinearFactorGraph graph;
  		int i = 0;
  		auto ss = plant_ptr -> get_state_space();
  		auto ds = plant_ptr -> get_derivative_space();
  		// double dt = 0.1;
		// for (int i = 0; i < ss -> get_dimension(); ++i)
  // 		{
  			// std::string str_S("Si");
  			// std::string str_D("Di");
  			// std::string str_U("Ui");

  			auto x0_key  = prx_symbol_t::state_symbol(t);
  			// auto xd0_key = prx_symbol_t::state_symbol(str_D, i, t);

  			auto x1_key  = prx_symbol_t::state_symbol(t + 1);
  			// auto xd1_key = prx_symbol_t::state_symbol(str_D, i, t + 1);

  			auto u1_key = prx_symbol_t::control_symbol(t);
  			// xd1_key = state_symbol(std::string('D' + static_cast<char>(i + 'a')), i, t + 1);

  			// gtsam::Double_ x0_expr(x0_key);
  			// gtsam::Double_ x1_expr(x1_key);
  			// gtsam::Double_ v0_expr(xd0_key);
  			// xd0_key.print("j: " + std::to_string(i) + "\t");
  			// Double_ v1_expr(xd1_key);
  			// std::cout << "prop factor: ( " << t << ", " << t+1 << ", " << t << " )\n";
  			graph.add(
  					propagation_factor_t(dynamics_model, 
  						x0_key, x1_key, 
  						u1_key, 
  						plant_ptr)
  					// gtsam::ExpressionFactor<double>(dynamics_model, 0.0, x0_expr + dt * v0_expr - x1_expr)
  				);
  			// i++;
  		// }
  		return graph;
	}

	gtsam::NonlinearFactorGraph trajectory_fg_t::add_goal_distance_factor(const int t)
	{
  		auto xi_f = prx_symbol_t::state_symbol(t);
		// auto xg_f = prx_symbol_t::goal_state_symbol();

  		gtsam::NonlinearFactorGraph graph;

  		graph.add(
  			goal_distance_factor_t(nullptr, 
  				xi_f, 
  				goal_state,
  				plant_ptr, 
  				t)
  		);

  		return graph;
	}

	gtsam::NonlinearFactorGraph trajectory_fg_t::get_fg(const int num_steps)
    {
  		gtsam::NonlinearFactorGraph graph;
  		std::cout << "num_steps: " << num_steps << std::endl;

  		graph.addPrior(
  			prx_symbol_t::state_symbol(0),
  			initial_state,
  			dynamics_model
  			);

  		// graph.addPrior(
  		// 	prx_symbol_t::goal_state_symbol(),
  		// 	goal_state,
  		// 	dynamics_model
  		// 	);

  		auto cs = plant_ptr -> get_control_space();

  		for (int t = 0; t < num_steps + 1; t++) 
  		{
  			graph.add(limits_factors(t, num_steps));
  	  		if (t < num_steps) 
  	  		{

  	    		graph.add(collocationFactors(t));
  	  		}
  	  		// else
  	  		// {
  	  		// 	graph.add(add_goal_distance_factor(t));
  	  		// }
  	  		graph.add(add_goal_distance_factor(t));

  		}
  		// std::string str_SN("Si");
  		// // str_S.push_back(static_cast<char>(i + 'a'));
  		// graph.addPrior(
  		// 	prx_symbol_t::state_symbol(str_SN, i, num_steps),
  		// 	goal_state,
  		// 	dynamics_model
  		// 	);
  		
  		// Eigen::VectorXd vs = gtsam::Vector::Zero(x_dim);
  		// vs[0] = M_PI;
  		// values.insert(xi_sy, vs);

  		return graph;
	}


	// gtsam::Values trajectory_fg_t::ZeroValues(const int t, const int num_steps)
	// {
	// 	gtsam::Values values;

	// 	auto sampler_noise_model =
 //      		gtsam::noiseModel::Isotropic::Sigma(
 //      			plant_ptr -> get_state_space() -> get_dimension(),
 //      			0);
 //  		gtsam::Sampler sampler(sampler_noise_model);
 //  		// auto sampler_noise_model =
 //  		//     gtsam::noiseModel::Isotropic::Sigma(6, gaussian_noise);
 //  		// Sampler sampler(sampler_noise_model);
	// 	int i = 0;
 //  		// Initialize link dynamics to 0.
 //  		// auto ss = plant_ptr -> get_state_space();
 //  		// for (int i = 0; i < ss -> get_dimension(); ++i)
 //  		// {
 //  			// std::string str_S("Si");
 //  			// std::string str_D("Di");
 //  			// std::string str_U("Ui");

 //  			auto x_dim = plant_ptr -> get_state_space() -> get_dimension();
 //  			auto u_dim = plant_ptr -> get_control_space() -> get_dimension();

 //  			auto xi_sy = prx_symbol_t::state_symbol(t);
 //  			Eigen::VectorXd vs = gtsam::Vector::Zero(x_dim);
 //  			// values.insert(xi_sy, vs);
 //  			values.insert(xi_sy, sampler.sample());

 //  			if (t < num_steps)
 //  			{
 //  				// auto xdi_sy = prx_symbol_t::state_symbol(str_D, i, t+1);
 //  				// Eigen::VectorXd vd = gtsam::Vector::Zero(x_dim);
 //  				// values.insert(xdi_sy, vd);

 //  				auto ui_sy = prx_symbol_t::control_symbol(t);
 //  				Eigen::VectorXd vu = gtsam::Vector::Zero(u_dim);
 //  				// eig
 //  				values.insert(ui_sy, vu);
 //  				// values.insert(ui_sy, sampler.sample());
 //  			}
 //  		// }

	// 	return values;
	// }

// 	gtsam::Values trajectory_fg_t::init_from_traj(const trajectory_t& traj, const plan_t& plan)
// 	{
// 		gtsam::Values values;

// 		auto ss = plant_ptr -> get_state_space();
// 		auto cs = plant_ptr -> get_control_space();

//   		auto x_dim = ss -> get_dimension();
//   		auto u_dim = cs -> get_dimension();

//   		// std::string str_S("Si");
//   		// std::string str_U("Ui");
//   		int t = 0;
// 		for (auto state : traj)
// 		// for (unsigned i = 0; i < traj.get_num_states(); ++i)
// 		{
// 			// auto state = traj[i];
// 			// std::cout << "t: " << t << std::endl;
  			
//   			auto xi_sy = prx_symbol_t::state_symbol(t);
//   			Eigen::VectorXd vs = gtsam::Vector::Zero(x_dim);
//   			ss -> copy_vector_from_point(vs, state);
//   			values.insert(xi_sy, vs);

//   		// 	if (i < traj.get_num_states() - 1)
//   		// 	{
// 				// auto ctrl  = plan[i];
// 				// auto ui_sy = prx_symbol_t::state_symbol(str_U, 0, t);
// 				// Eigen::VectorXd vu = gtsam::Vector::Zero(u_dim);
//   		 		// ss -> copy_vector_from_point(vu, ctrl);
// 				// values.insert(ui_sy, vu);
//   		// 	}
// 			t++;
// 		}
// 		// auto xi_sy = prx_symbol_t::state_symbol(str_S, 0, t);
//   // 		Eigen::VectorXd vs = gtsam::Vector::Zero(x_dim);
//   // 		vs[0] = M_PI;
//   // 		values.insert(xi_sy, vs);
		

// 		t = 0; 
// 		for (auto step : plan)
// 		{
// 			std::cout << "t: " << t << std::endl;

// 			auto ui_sy = prx_symbol_t::control_symbol(t);
// 			Eigen::VectorXd vu = gtsam::Vector::Zero(u_dim);
// // PRX_DEBUG_PRINT
//   			cs -> copy_vector_from_point(vu, step.control);
// // PRX_DEBUG_PRINT
// 			values.insert(ui_sy, vu);
// 			t++;
// 		}


//   		// values.print("Values", prx_key_formatter);
// 		return values;
// 	}
// 	gtsam::Values trajectory_fg_t::ZeroValuesTrajectory(const int num_steps) 
// 	{
//   		gtsam::Values z_values;
//   		for (int t = 0; t <= num_steps; t++)
// 	  	{
// 	   		z_values.insert(ZeroValues(t, num_steps));
//   		}
//   		return z_values;
// 	}
}