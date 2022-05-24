#include "prx/gtdynamics/utilities/utilities_functions.hpp"
#include "prx/gtdynamics/utilities/fg_logger.hpp"

namespace prx
{
	namespace fg_utilities
	{	

		const gtsam::Values& optimize_and_log(
			gtsam::NonlinearOptimizer& nl_opt, 
			const gtsam::NonlinearOptimizerParams& params,
			fg_logger_t& logger, const int extra_iters) 
		{ 
			double currentError = nl_opt.error();

			// check if we're already close enough
			if (currentError <= params.errorTol) 
			{
				if (params.verbosity >= gtsam::NonlinearOptimizerParams::ERROR)
					std::cout << "Exiting, as error = " << currentError << " < " << params.errorTol << std::endl;
				return nl_opt.values();
			}

			// Maybe show output
			if (params.verbosity >= gtsam::NonlinearOptimizerParams::VALUES)
				nl_opt.values().print("Initial values");
			if (params.verbosity >= gtsam::NonlinearOptimizerParams::ERROR)
				std::cout << "Initial error: " << currentError << std::endl;

			// Return if we already have too many iterations
			if (nl_opt.iterations() >= params.maxIterations) 
			{
				if (params.verbosity >= gtsam::NonlinearOptimizerParams::TERMINATION) 
				{
					std::cout << "iterations: " << nl_opt.iterations() << " >? " << params.maxIterations << std::endl;
				}
				return nl_opt.values();
			}

			// auto ss = plant -> get_state_space();
			// auto cs = plant -> get_control_space();
			// std::shared_ptr<plan_t> sln_plan = std::make_shared<plan_t>(cs);
			// std::shared_ptr<trajectory_t> sln_traj = std::make_shared<trajectory_t>(ss);
			// Iterative loop
			double newError = currentError; // used to avoid repeated calls to error()
			do 
			{
				logger.add_graph_errors(nl_opt.graph(), nl_opt.values(), std::to_string(extra_iters + nl_opt.iterations()));
				
				// Do next iteration
				currentError = newError;
				nl_opt.iterate();
				gtsam::tictoc_finishedIteration_();

				// Update newError for either printouts or conditional-end checks:
				newError = nl_opt.error();

				// User hook:
				if (params.iterationHook)
					params.iterationHook(nl_opt.iterations(), currentError, newError);

				// Maybe show output
				if (params.verbosity >= gtsam::NonlinearOptimizerParams::VALUES)
					nl_opt.values().print("newValues");
				if (params.verbosity >= gtsam::NonlinearOptimizerParams::ERROR)
					std::cout << "newError: " << newError << std::endl;

			} 
			while (nl_opt.iterations() < params.maxIterations &&
					!gtsam::checkConvergence(params.relativeErrorTol, params.absoluteErrorTol, params.errorTol,
							 currentError, newError, params.verbosity) && 
					std::isfinite(currentError));

			logger.add_graph_errors(nl_opt.graph(), nl_opt.values(), std::to_string(extra_iters + nl_opt.iterations()));
			
			// Printing if verbose
			if (params.verbosity >= gtsam::NonlinearOptimizerParams::TERMINATION) 
			{
				std::cout << "iterations: " << nl_opt.iterations() << " >? " << params.maxIterations << std::endl;
				if (nl_opt.iterations() >= params.maxIterations)
					std::cout << "Terminating because reached maximum iterations" << std::endl;
			}

			return nl_opt.values(); 
		}

		void values_to_plan(const gtsam::Values& vals, plan_t* plan, const int total_steps)
		{
			plan -> clear();

			int ti = 0;

			// X, U \in [0, T)
			// for (double t_elapsed = 0; ti < total_steps; ti++, t_elapsed += simulation_step) 
			for (int i = 0; i < total_steps; ++i)
			{
				// auto ti = symbol_factory_t::create_symbol("time_symbol", i);
				auto us = symbol_factory_t::create_symbol("control_symbol", i);

				auto u = vals.at<Eigen::VectorXd>(us);
				// auto t = vals.at<Eigen::VectorXd>(ti);
				plan -> copy_onto_back(u, 1);
			}
		}

		void values_to_traj(const gtsam::Values& vals, trajectory_t& traj, const int total_steps)
		{
			traj.clear();

			int ti = 0;

			// X, U \in [0, T)
			for (double t_elapsed = 0; ti < total_steps; ti++, t_elapsed += simulation_step) 
			{
				auto xs = symbol_factory_t::create_symbol("state_symbol", ti);				
				auto x = vals.at<Eigen::VectorXd>(xs);
				traj.copy_onto_back(x);
			}
		}

		void values_to_plan_and_traj(const gtsam::Values& vals, trajectory_t* traj, plan_t* plan, const int total_steps)
		{
			traj -> clear();
			plan -> clear();
			
			int ti = 0;

			// X, U \in [0, T)
			for (double t_elapsed = 0; ti < total_steps; ti++, t_elapsed += simulation_step) 
			{
				auto xs = symbol_factory_t::create_symbol("state_symbol", ti);
				auto us = symbol_factory_t::create_symbol("control_symbol", ti);
				
				auto x = vals.at<Eigen::VectorXd>(xs);
				auto u = vals.at<Eigen::VectorXd>(us);

				traj -> copy_onto_back(x);
				plan -> copy_onto_back(u, simulation_step);

			}

			// Append the last state, at time T
			auto xs = symbol_factory_t::create_symbol("state_symbol", ti);
			auto x = vals.at<Eigen::VectorXd>(xs);
			traj -> copy_onto_back(x);
			
		}

		void updates_values_from_plan_and_traj(gtsam::Values& values, system_ptr_t _sys_ptr, const trajectory_t& traj, const plan_t& plan, const int total_steps)
		{
			prx_assert(traj.size() == total_steps + 1, "Mismatch on size of state_symbols " << total_steps << " and trajectory_t states " << traj.size() << ".");
			
			unsigned t = 0;
			auto ss = _sys_ptr -> get_state_space();
			auto cs = _sys_ptr -> get_control_space();
			auto x_dim = ss -> get_dimension();
			auto u_dim = cs -> get_dimension();
	
			Eigen::VectorXd vs = gtsam::Vector::Zero(x_dim);
			Eigen::VectorXd vu = gtsam::Vector::Zero(u_dim);

			for (auto state : traj)
			{
				auto xs = symbol_factory_t::create_symbol("state_symbol", t);
				ss -> copy_vector_from_point(vs, state);
				// values.insert(xs, vs);

    			values.update(xs, vs);

				t++;
			}
			t = 0; 
			for (auto step : plan)
			{
				for (double i = 0; i < step.duration; i += simulation_step)
				{
					auto ui_sy = prx_symbol_t::control_symbol(t);
					cs -> copy_vector_from_point(vu, step.control);
					values.update(ui_sy, vu);
					t++;
				}
			}
		}

		void add_noise(gtsam::Values& values, const std::string symbol_name, boost::shared_ptr<gtsam::noiseModel::Isotropic> & noise)
		{
			std::function<bool(gtsam::Key)> fn = [&](gtsam::Key k)
			{
				auto ps = prx_symbol_t(k);
				return ps.label() == symbol_name && ps.time() > 0;
			};

			auto filtered = values.filter<Eigen::VectorXd>(fn);
  			gtsam::Sampler sampler(noise);

			for (auto v : filtered)
			{
				// std::cout << "filtered: " << prx::key_formatter(v.key) << std::endl;
				v.value = v.value + sampler.sample();
			}
		}


	}
}