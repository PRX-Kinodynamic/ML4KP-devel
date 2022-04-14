#include "prx/gtdynamics/utilities/utilities_functions.hpp"
#include "prx/gtdynamics/utilities/fg_logger.hpp"

namespace prx
{
	namespace fg_utilities
	{	

		const gtsam::Values& optimize_and_log(
			gtsam::NonlinearOptimizer& nl_opt, 
			const gtsam::NonlinearOptimizerParams& params,
			fg_logger_t& logger) 
		{ 
			double currentError = nl_opt.error();
			logger.add_graph_errors(nl_opt.graph(), nl_opt.values(), std::to_string(nl_opt.iterations()));

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

			// Iterative loop
			double newError = currentError; // used to avoid repeated calls to error()
			do 
			{
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

				logger.add_graph_errors(nl_opt.graph(), nl_opt.values(), std::to_string(nl_opt.iterations()));
			} 
			while (nl_opt.iterations() < params.maxIterations &&
					!gtsam::checkConvergence(params.relativeErrorTol, params.absoluteErrorTol, params.errorTol,
                             currentError, newError, params.verbosity) && 
					std::isfinite(currentError));

			// Printing if verbose
			if (params.verbosity >= gtsam::NonlinearOptimizerParams::TERMINATION) 
			{
				std::cout << "iterations: " << nl_opt.iterations() << " >? " << params.maxIterations << std::endl;
				if (nl_opt.iterations() >= params.maxIterations)
					std::cout << "Terminating because reached maximum iterations" << std::endl;
			}

			return nl_opt.values(); 
		}

		void values_to_traj(gtsam::Values& vals, trajectory_t& traj)
		{
  			for (int t = 0; t <= t_steps; t++, t_elapsed += dt) 

  			do
    		{
    		    // lqr.compute_controls();
    		    cs -> enforce_bounds();
    		    // sln_plan.append_onto_back(simulation_step, cs);
    		    plant -> propagate(simulation_step);
    		    
    		    traj.copy_onto_back(vec);
    		}
    		while(!checker.check());

		}

	}
}