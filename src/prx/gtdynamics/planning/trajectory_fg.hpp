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
#include "prx/gtdynamics/factors/space_limit_factor.hpp"
#include "prx/gtdynamics/factors/propagation_factor.hpp"
#include "prx/gtdynamics/factors/goal_distance_factor.hpp"

namespace prx
{
	class trajectory_fg_t
	{
	public:
		trajectory_fg_t(system_ptr_t);

		// set_x_factors(const std::vector<bool> _vec)
		// 	: x_ss(_vec)
		// 	{};

		// set_xdot_factors(const std::vector<bool> _vec)
		// 	: xdot_ss(_vec)
		// 	{};

			void set_initial_state(const std::vector<double> _vec)
			{
				initial_state.resize(_vec.size());
				for (int i = 0; i < _vec.size(); ++i)
				{
					initial_state[i] = _vec[i];
				}
			};

			void set_goal_state(const std::vector<double> _vec)
			{
				goal_state.resize(_vec.size());
				for (int i = 0; i < _vec.size(); ++i)
				{
					goal_state[i] = _vec[i];
				}
			};

		gtsam::NonlinearFactorGraph get_fg(const int num_steps);

		gtsam::NonlinearFactorGraph limits_factors(const int t, const int num_steps) const ;

		gtsam::NonlinearFactorGraph collocationFactors(const int t);
	
		gtsam::NonlinearFactorGraph add_goal_distance_factor(const int t);

		// gtsam::Values ZeroValues(const int t, const int num_steps);
		
		// gtsam::Values ZeroValuesTrajectory(const int num_steps);

		gtsam::Values init_from_traj(const trajectory_t& traj, const plan_t& plan);

		private:
			plant_ptr_t plant_ptr;
			Eigen::VectorXd initial_state;
			Eigen::VectorXd goal_state;
  			gtsam::noiseModel::Isotropic::shared_ptr dynamics_model;// = Isotropic::Sigma(1, 1e-5);    // Dynamics constraints.
  			gtsam::noiseModel::Isotropic::shared_ptr control_model;// = Isotropic::Sigma(1, 1e-5);    // Dynamics constraints.

	};
}