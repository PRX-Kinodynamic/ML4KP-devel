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
	
	gtsam::NonlinearFactorGraph trajectory_fg_t::add_state_propagation_factor(const int t)
	{

		// for (auto x : sys_ptr -> get_state_space())
		gtsam::NonlinearFactorGraph graph;
		int i = 0;
		auto ss = plant_ptr -> get_state_space();
		auto ss_dim = ss -> get_dimension();

		auto x0_key  = symbol_factory_t::create_symbol("state_symbol", t);
		auto x1_key  = prx_symbol_t::state_symbol(t + 1);
		auto u1_key = prx_symbol_t::control_symbol(t);

		for (int i = 0; i < ss_dim; ++i)
		{
			graph.add(
				state_propagation_factor_t(nullptr,
					x0_key, x1_key, 
					u1_key, i,
					plant_ptr
					)
				);
		}

		return graph;
	}

	gtsam::NonlinearFactorGraph trajectory_fg_t::add_propagation_factor(const int t, const int prop_type)
	{

		// for (auto x : sys_ptr -> get_state_space())
		gtsam::NonlinearFactorGraph graph;
		int i = 0;
		auto ss = plant_ptr -> get_state_space();
		auto ds = plant_ptr -> get_derivative_space();

		auto x0_key  = prx_symbol_t::state_symbol(t);

		auto x1_key  = prx_symbol_t::state_symbol(t + 1);

		auto u1_key = prx_symbol_t::control_symbol(t);

		
		if (prop_type == 3)
		{
			graph.add(
				propagation_factor_t(dynamics_model, 
					x0_key, x1_key, 
					u1_key, 
					plant_ptr)
				);
		}
		else if (prop_type == 4)
		{
			auto t01_key = prx_symbol_t::time_symbol(t);
			graph.add(
				propagation_factor_4_t(dynamics_model, 
					x0_key, x1_key, 
					u1_key, t01_key,
					plant_ptr)
				);
		}
		else if (prop_type == 5)
		{
			graph.add(add_state_propagation_factor(t));
		}
		else
		{
			prx_throw("Propagation type not supported.");
		}

		return graph;
	}

	gtsam::NonlinearFactorGraph trajectory_fg_t::add_energy_factors(const int t, goal_factor_params_t params)
	{
		auto xi = symbol_factory_t::create_symbol("state_symbol", t);
		gtsam::NonlinearFactorGraph graph;

		// goal_factor_params_t params;
		// params.goal = goal_state;
		graph.add(
			kinetic_energy_factor_t(nullptr, 
				xi, 
				plant_ptr,
				t,
				params)
		);

		graph.add(
			potential_energy_factor_t(nullptr, 
				xi, 
				plant_ptr,
				params.T - t,
				params
				)
		);

		return graph;
	}

	gtsam::NonlinearFactorGraph trajectory_fg_t::add_goal_distance_factor(const int t, const goal_factor_params_t& _params)
	// const tot_steps, const double theta, Eigen::VectorXd& error_scale)
	{
		auto xi_f = prx_symbol_t::state_symbol(t);

		gtsam::NonlinearFactorGraph graph;

		graph.add(
			goal_distance_factor_t(nullptr, 
				xi_f, 
				plant_ptr,
				t,
				_params)
		);

		return graph;
	}

	gtsam::NonlinearFactorGraph trajectory_fg_t::get_fg(
		const trajectory_fg_params_t& _params)
	{
		gtsam::NonlinearFactorGraph graph;
		int num_steps = _params.num_steps;
		// std::cout << "num_steps: " << num_steps << std::endl;

		if (_params.initial_state_as_prior)
		{
			graph.addPrior(
				prx_symbol_t::state_symbol(0),
				initial_state,
				dynamics_model
				);
		}
		if (_params.goal_state_as_prior)
		{
			graph.addPrior(
				prx_symbol_t::state_symbol(num_steps),
				goal_state,
				dynamics_model
				);
		}

		auto cs = plant_ptr -> get_control_space();

		for (int t = 0; t < num_steps + 1; t++) 
		{
			if (_params.limits_factors) graph.add(limits_factors(t, num_steps));
			if (t < num_steps) 
			{
				// case(3): graph.add(collocationFactors(t)); break;
				graph.add(add_propagation_factor(t, _params.propagation_factors_type));
				// graph.add(add_state_propagation_factor(t));
			}

			if (_params.use_goal_factors) graph.add(add_goal_distance_factor(t, _params.goal_factor_params));
			if (_params.use_energy_factors) graph.add(add_energy_factors(t, _params.goal_factor_params));
		}

		return graph;
	}

}