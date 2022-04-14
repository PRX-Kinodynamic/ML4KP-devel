#pragma once 

#include "prx/utilities/defs.hpp"

#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>


// #define GET_FACTOR_NAME_MACRO_GTSAM(FACTOR_CLASS, FACTOR, NAME) \
// 	if (dynamic_cast<gtsam::FACTOR_CLASS*>(FACTOR.get())) return NAME;

namespace prx
{
	class fg_logger_t;
	namespace fg_utilities
	{	

		const gtsam::Values& optimize_and_log(
			gtsam::NonlinearOptimizer& nl_opt, const gtsam::NonlinearOptimizerParams& params, fg_logger_t& logger);

		void values_to_traj(gtsam::Values& vals, trajectory_t& traj);

	}
}