#pragma once

#include "prx/utilities/defs.hpp"

#include "prx/simulation/general/condition_check.hpp"
#include "prx/simulation/playback/plan.hpp"
#include "prx/simulation/playback/trajectory.hpp"
#include "prx/simulation/system.hpp"

#include <gtsam/linear/Sampler.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/linear/NoiseModel.h>

// #define GET_FACTOR_NAME_MACRO_GTSAM(FACTOR_CLASS, FACTOR, NAME) \
// 	if (dynamic_cast<gtsam::FACTOR_CLASS*>(FACTOR.get())) return NAME;

namespace prx
{
class fg_logger_t;
namespace fg_utilities
{

const gtsam::Values& optimize_and_log(gtsam::NonlinearOptimizer& nl_opt, const gtsam::NonlinearOptimizerParams& params,
                                      fg_logger_t& logger, const int extra_iters = 0,
                                      condition_check_t* checker_0 = nullptr);

void values_to_plan_and_traj(const gtsam::Values& vals, trajectory_t* traj, plan_t* plan, const int total_steps);
// void values_to_plan_and_traj(gtsam::Values& vals, trajectory_t* traj, plan_t* plan, const double duration);

void updates_values_from_plan_and_traj(gtsam::Values& values, system_ptr_t _sys_ptr, const trajectory_t& traj,
                                       const plan_t& plan, const int total_steps);

void add_noise(gtsam::Values& values, const std::string symbol_name,
               boost::shared_ptr<gtsam::noiseModel::Isotropic>& noise);

void values_to_plan(const gtsam::Values& vals, plan_t* plan, const int total_steps);

void values_to_traj(const gtsam::Values& vals, trajectory_t& traj, const int total_steps);

}  // namespace fg_utilities
}  // namespace prx