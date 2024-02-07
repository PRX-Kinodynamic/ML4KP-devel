#include "prx/factor_graphs/utilities/common_functions.hpp"

namespace prx
{
namespace fg
{

const gtsam::Values& optimize_and_log(gtsam::NonlinearOptimizer& nl_opt, const gtsam::NonlinearOptimizerParams& params,
                                      condition_check_t* checker_0)
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

  // Iterative loop
  double newError = currentError;  // used to avoid repeated calls to error()

  custom_check_t condition_1 = [&]() {
    return gtsam::checkConvergence(params.relativeErrorTol, params.absoluteErrorTol, params.errorTol, currentError,
                                   newError, params.verbosity);
  };
  custom_check_t condition_2 = [&]() { return !std::isfinite(currentError); };

  condition_check_t checker_1(condition_1);
  condition_check_t checker_2(condition_2);
  condition_check_t checker_all("iterations", params.maxIterations);

  if (checker_0 != nullptr)
  {
    checker_all.add_condition(checker_0);
  }
  checker_all.add_condition(&checker_1);
  checker_all.add_condition(&checker_2);

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

  } while (!checker_all.check());

  // Printing if verbose
  if (params.verbosity >= gtsam::NonlinearOptimizerParams::TERMINATION)
  {
    std::cout << "iterations: " << nl_opt.iterations() << " >? " << params.maxIterations << std::endl;
    if (nl_opt.iterations() >= params.maxIterations)
      std::cout << "Terminating because reached maximum iterations" << std::endl;
  }

  return nl_opt.values();
}

}  // namespace fg
}  // namespace prx