#include "prx/gtdynamics/planning/trajectory_fg.hpp"

namespace prx
{
	trajectory_fg_t::trajectory_fg_t(system_ptr_t _sys_ptr)
	{
		prx_assert(_sys_ptr != nullptr, "trajectory_fg_t cannot handle nullptr systems!");
		plant_ptr = std::dynamic_pointer_cast<plant_t>(_sys_ptr);
		prx_assert(plant_ptr != nullptr, "A type plant_t is needed!");
		// sys_ptr = _sys_ptr;
		dynamics_model = gtsam::noiseModel::Isotropic::Sigma(
			plant_ptr -> get_state_space() -> get_dimension(),
			1e-5);
		control_model = gtsam::noiseModel::Isotropic::Sigma(
			plant_ptr -> get_control_space() -> get_dimension(),
			1e-5);    // Dynamics constraints.
	}

	gtsam::NonlinearFactorGraph trajectory_fg_t::limits_factors(const int t) const 
    {
  		gtsam::NonlinearFactorGraph graph;

  		auto ss = plant_ptr -> get_state_space();
  		for (int i = 0; i < ss -> get_dimension(); ++i)
  		{
// using CustomErrorFunction = std::function<Vector(const CustomFactor &, const Values &, const JacobianVector *)>;
  			// gtsam::CustomErrorFunction range_fn = [](const CustomFactor& cf, const Values& v, const JacobianVector* jv)
  			// {
  			// }
  			std::string str("Si");
  			// str.push_back(static_cast<char>(i + 'a'));
  			auto st_sy = prx_symbol_t::state_symbol(str, i, t);
  			// Double_ x0_expr(st_sy);

  			graph.add(
  			// 	CustomFactor(
  			// 		dynamics_model, 
  			// 		
  			// 		)
  				state_limit_factor(st_sy, dynamics_model, 
  					ss -> get_lower_bound(i),
  					ss -> get_upper_bound(i)
  					)
  				);
  			// i++;

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
  		double dt = 0.1;
		// for (int i = 0; i < ss -> get_dimension(); ++i)
  // 		{
  			std::string str_S("Si");
  			// str_S.push_back(static_cast<char>(i + 'a'));
  			std::string str_D("Di");
  			std::string str_U("Ui");
  			// str_D.push_back(static_cast<char>(i + 'a'));

  			auto x0_key  = prx_symbol_t::state_symbol(str_S, i, t);
  			auto xd0_key = prx_symbol_t::state_symbol(str_D, i, t);

  			auto x1_key  = prx_symbol_t::state_symbol(str_S, i, t + 1);
  			auto xd1_key = prx_symbol_t::state_symbol(str_D, i, t + 1);

  			auto u1_key = prx_symbol_t::state_symbol(str_U, i, t + 1);
  			// xd1_key = state_symbol(std::string('D' + static_cast<char>(i + 'a')), i, t + 1);

  			// gtsam::Double_ x0_expr(x0_key);
  			// gtsam::Double_ x1_expr(x1_key);
  			// gtsam::Double_ v0_expr(xd0_key);
  			// xd0_key.print("j: " + std::to_string(i) + "\t");
  			// Double_ v1_expr(xd1_key);
  			graph.add(
  					propagation_factor_t(dynamics_model, x0_key, x1_key, xd1_key, u1_key, plant_ptr)
  					// gtsam::ExpressionFactor<double>(dynamics_model, 0.0, x0_expr + dt * v0_expr - x1_expr)
  				);
  			// i++;
  		// }
  		return graph;
	}

	gtsam::NonlinearFactorGraph trajectory_fg_t::get_fg(const int num_steps)
    {
  		gtsam::NonlinearFactorGraph graph;

  		int i = 0;
  		// Eigen::VectorXd x(ltv -> get_state_space() -> get_dimension());

  		std::string str_S0("Si");
  		graph.addPrior(
  			prx_symbol_t::state_symbol(str_S0, i, 0),
  			initial_state,
  			dynamics_model
  			);

  		i = 0;

  		std::string str_SN("Si");
  		// str_S.push_back(static_cast<char>(i + 'a'));
  		graph.addPrior(
  			prx_symbol_t::state_symbol(str_SN, i, num_steps),
  			goal_state,
  			dynamics_model
  			);
  		// 	i++;
  		// }

  		auto cs = plant_ptr -> get_control_space();

  		for (int t = 0; t < num_steps + 1; t++) 
  		{
  			// graph.add(limits_factors(t));
  	  		if (t < num_steps) 
  	  		{
  	  			// i = 0;
  	  			int j = 0;
  	  	// 		for (int j = 0; j < cs -> get_dimension(); ++j)
  				// {
  					// std::string str_U("Ui");
  					// str_U.push_back(static_cast<char>(j + 'a'));
  					// auto u_k = prx_symbol_t::state_symbol(str_U, j, t+1);
  					
  					// u_k.print("j: " + std::to_string(j) + "\t");  					
    				// graph.emplace_shared<gtdynamics::MinTorqueFactor>(
    				// 	u_k,
    				// 	control_model
    				// 	);
    				// i++;
  				// }
  	    		graph.add(collocationFactors(t));
  	  		}
  		}

  		// for (int t = 0; t <= num_steps; t++)
  		// {
  		// 	// for (auto ctrl : cs)
  	 //  		for (int j = 0; j < cs -> get_dimension(); ++j)
  		// 	{
  		// 		std::string str_U("U");
  		// 		str_U.push_back(static_cast<char>(j + 'a'));
    // 			graph.emplace_shared<gtdynamics::MinTorqueFactor>(
    // 				prx_symbol_t::state_symbol(str_U, j, t),
    // 				dynamics_model
    // 				);
  		// 	}
  		// }

  		return graph;
	}


	gtsam::Values trajectory_fg_t::ZeroValues(const int t, const int num_steps)
	{
		gtsam::Values values;

  		// auto sampler_noise_model =
  		//     gtsam::noiseModel::Isotropic::Sigma(6, gaussian_noise);
  		// Sampler sampler(sampler_noise_model);
		int i = 0;
  		// Initialize link dynamics to 0.
  		// auto ss = plant_ptr -> get_state_space();
  		// for (int i = 0; i < ss -> get_dimension(); ++i)
  		// {
  			std::string str_S("Si");
  			// str_S.push_back(static_cast<char>(i + 'a'));

  			std::string str_D("Di");
  			// str_D.push_back(static_cast<char>(i + 'a'));

  			std::string str_U("Ui");
  			// str_U.push_back(static_cast<char>(i + 'a'));

  			auto x_dim = plant_ptr -> get_state_space() -> get_dimension();
  			auto u_dim = plant_ptr -> get_control_space() -> get_dimension();

  			auto xi_sy = prx_symbol_t::state_symbol(str_S, i, t);
  			Eigen::VectorXd vs = gtsam::Vector::Zero(x_dim);
  			values.insert(xi_sy, vs);

  			if (t < num_steps)
  			{
  				auto xdi_sy = prx_symbol_t::state_symbol(str_D, i, t+1);
  				Eigen::VectorXd vd = gtsam::Vector::Zero(x_dim);
  				values.insert(xdi_sy, vd);

  				auto ui_sy = prx_symbol_t::state_symbol(str_U, i, t+1);
  				Eigen::VectorXd vu = gtsam::Vector::Zero(u_dim);
  				values.insert(ui_sy, vu);
  			}
  		// }

		return values;
	}

	gtsam::Values trajectory_fg_t::ZeroValuesTrajectory(const int num_steps) 
	{
  		gtsam::Values z_values;
  		for (int t = 0; t <= num_steps; t++)
	  	{
	   		z_values.insert(ZeroValues(t, num_steps));
  		}
  		return z_values;
	}
}