#pragma once
#include "gtsam/nonlinear/LevenbergMarquardtParams.h"

namespace prx
{
namespace fg
{
namespace utilities
{

double angle_diff(const double& a0, const double& a1)
{
  return std::min((2 * PRX_PI) - std::fabs(a0 - a1), std::fabs(a0 - a1));
}

gtsam::LevenbergMarquardtParams default_levenberg_marquardt_parameters()
{
  gtsam::LevenbergMarquardtParams lm_params;
  lm_params.setVerbosityLM("SUMMARY");
  lm_params.setlambdaUpperBound(1e32);
  lm_params.setUseFixedLambdaFactor(false);
  lm_params.setDiagonalDamping(true);
  lm_params.setlambdaFactor(2);
  lm_params.setlambdaInitial(1e-7);
  lm_params.setMaxIterations(10);
  lm_params.setRelativeErrorTol(1e-6);
  lm_params.setAbsoluteErrorTol(1e-6);
  return lm_params;
}

gtsam::LevenbergMarquardtParams levenberg_marquardt_parameters(const prx::param_loader& params,
                                                               const bool verbose = true)
{
  gtsam::LevenbergMarquardtParams lm_params;
  if (params.exists("verbosity"))
  {
    lm_params.setVerbosityLM(params["verbosity"].as<>());
    if (verbose)
      std::cout << "Set verbosity to " << params["verbosity"].as<>() << std::endl;
  }
  if (params.exists("lambda_upper_bound"))
  {
    lm_params.setlambdaUpperBound(params["lambda_upper_bound"].as<double>());
    if (verbose)
      std::cout << "Set lambda_upper_bound to " << params["lambda_upper_bound"].as<>() << std::endl;
  }
  if (params.exists("use_fixed_lambda_factor"))
  {
    lm_params.setUseFixedLambdaFactor(params["use_fixed_lambda_factor"].as<bool>());
    if (verbose)
      std::cout << "Set use_fixed_lambda_factor to " << params["use_fixed_lambda_factor"].as<>() << std::endl;
  }
  if (params.exists("diagonal_damping"))
  {
    lm_params.setDiagonalDamping(params["diagonal_damping"].as<bool>());
    if (verbose)
      std::cout << "Set diagonal_damping to " << params["diagonal_damping"].as<>() << std::endl;
  }
  if (params.exists("lambda_factor"))
  {
    lm_params.setlambdaFactor(params["lambda_factor"].as<double>());
    if (verbose)
      std::cout << "Set lambda_factor to " << params["lambda_factor"].as<>() << std::endl;
  }
  if (params.exists("lambda_initial"))
  {
    lm_params.setlambdaInitial(params["lambda_initial"].as<double>());
    if (verbose)
      std::cout << "Set lambda_initial to " << params["lambda_initial"].as<>() << std::endl;
  }
  if (params.exists("lambda_lower_bound"))
  {
    lm_params.setlambdaLowerBound(params["lambda_lower_bound"].as<double>());
    if (verbose)
      std::cout << "Set lambda_lower_bound to " << params["lambda_lower_bound"].as<>() << std::endl;
  }
  if (params.exists("log_file"))
  {
    lm_params.setLogFile(params["log_file"].as<>());
    if (verbose)
      std::cout << "Set log_file to " << params["log_file"].as<>() << std::endl;
  }
  if (params.exists("error_tolerance"))
  {
    lm_params.setErrorTol(params["error_tolerance"].as<double>());
    if (verbose)
      std::cout << "Set error_tolerance to " << params["error_tolerance"].as<>() << std::endl;
  }
  if (params.exists("relative_error_tolerance"))
  {
    lm_params.setRelativeErrorTol(params["relative_error_tolerance"].as<double>());
    if (verbose)
      std::cout << "Set relative_error_tolerance to " << params["relative_error_tolerance"].as<>() << std::endl;
  }
  if (params.exists("absolute_error_tolerance"))
  {
    lm_params.setAbsoluteErrorTol(params["absolute_error_tolerance"].as<double>());
    if (verbose)
      std::cout << "Set absolute_error_tolerance to " << params["absolute_error_tolerance"].as<>() << std::endl;
  }
  if (params.exists("max_iterations"))
  {
    lm_params.setMaxIterations(params["max_iterations"].as<int>());
    if (verbose)
      std::cout << "Set max_iterations to " << params["max_iterations"].as<>() << std::endl;
  }
  return lm_params;
}

}  // namespace utilities
}  // namespace fg
}  // namespace prx