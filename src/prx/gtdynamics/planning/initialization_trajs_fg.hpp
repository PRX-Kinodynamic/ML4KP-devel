#pragma once

#include <gtsam/linear/Sampler.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <gtdynamics/factors/MinTorqueFactor.h>

#include "prx/simulation/plant.hpp"
#include "prx/simulation/playback/plan.hpp"
#include "prx/utilities/general/zipped_iter.hpp"
#include "prx/simulation/playback/trajectory.hpp"

#include "prx/gtdynamics/utilities/prx_symbols.hpp"

namespace prx
{
	class initialization_trajs_fg_t
	{
		public:
			static gtsam::Values zero_state(const system_ptr_t _sys_ptr, const int t, const int num_steps, const double sigma);
			
			static gtsam::Values zeros_trajectory(const system_ptr_t _sys_ptr, const int num_steps, const double sigma = 0.0);
		
			static gtsam::Values state_from_space(const space_t* space, const int t, const bool state_or_ctrl, const double sigma);
			
			static gtsam::Values constant_trajectory(const system_ptr_t _sys_ptr, const space_point_t x, const space_point_t u, const int num_steps, const double sigma = 0.0);
			
			static gtsam::Values linear_trajectory(const system_ptr_t _sys_ptr, 
				const space_point_t& start, const space_point_t& goal,
				const int num_steps);

			static gtsam::Values init_from_traj(const system_ptr_t _sys_ptr, const trajectory_t& traj, const plan_t& plan);

			static gtsam::Values control_to_value(const system_ptr_t _sys_ptr, const space_point_t pt, const int t);

			static gtsam::Values state_to_value(const system_ptr_t _sys_ptr, const space_point_t pt, const int t);

			static gtsam::Values init_time_factors(const int num_steps, const double sigma);

	};
}