#pragma once

#include <gtsam/linear/Sampler.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <gtdynamics/factors/MinTorqueFactor.h>

#include "prx/simulation/plant.hpp"
#include "prx/simulation/playback/plan.hpp"
#include "prx/utilities/general/zipped_iter.hpp"
#include "prx/simulation/playback/trajectory.hpp"

#include "prx/gtdynamics/factors/factors.hpp"
#include "prx/gtdynamics/utilities/prx_symbols.hpp"
#include "prx/gtdynamics/utilities/symbols_factory.hpp"
namespace prx
{
	struct trajectory_fg_params_t
	{
		int  num_steps = 50;
		bool initial_state_as_prior = true;
		bool goal_state_as_prior = false;
		bool limits_factors = true;
		int  propagation_factors_type = 3; // only 3 or 4 for now... could be an enum
		bool use_goal_factors = true;
		bool use_energy_factors = false;
		goal_factor_params_t goal_factor_params;

		void print()
		{
			std::cout << "trajectory_fg_params_t:" << std::endl;
			std::cout << "\tnum_steps:" << num_steps << std::endl;
			std::cout << "\tinitial_state_as_prior:" << initial_state_as_prior << std::endl;
			std::cout << "\tgoal_state_as_prior:" << goal_state_as_prior << std::endl;
			std::cout << "\tlimits_factors:" << limits_factors << std::endl;
			std::cout << "\tpropagation_factors_type:" << propagation_factors_type << std::endl;
			std::cout << "\tuse_goal_factors:" << use_goal_factors << std::endl;
			std::cout << "\tuse_energy_factors:" << use_energy_factors << std::endl;
			std::cout << goal_factor_params << std::endl;
			// std::cout << "\t:" << std::endl;
		}
	};

	class trajectory_fg_t
	{
		public:
			trajectory_fg_t(system_ptr_t);

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
				// std::cout << "_vec: " << _vec.size() << std::endl;
				// std::cout << "goal: " << goal_state.transpose() << std::endl;
				for (int i = 0; i < _vec.size(); ++i)
				{
					goal_state[i] = _vec[i];
				}
			};

		gtsam::NonlinearFactorGraph get_fg(const trajectory_fg_params_t& _params);
		
		gtsam::NonlinearFactorGraph get_smoothing_fg(const trajectory_fg_params_t& _params);

		gtsam::NonlinearFactorGraph limits_factors(const int t, const int num_steps) const ;

		gtsam::NonlinearFactorGraph add_propagation_factor(const int t, const int prop_type);
	
		gtsam::NonlinearFactorGraph add_goal_distance_factor(const int t, const goal_factor_params_t& _params);

		gtsam::NonlinearFactorGraph add_energy_factors(const int t, goal_factor_params_t params);

		gtsam::NonlinearFactorGraph add_state_propagation_factor(const int t);

		gtsam::Values init_from_traj(const trajectory_t& traj, const plan_t& plan);

		gtsam::NonlinearFactorGraph add_bang_bang_factor(const int t);

		private:
			plant_ptr_t plant_ptr;
			Eigen::VectorXd initial_state;
			Eigen::VectorXd goal_state;
  			gtsam::noiseModel::Isotropic::shared_ptr dynamics_model;// = Isotropic::Sigma(1, 1e-5);    // Dynamics constraints.
  			gtsam::noiseModel::Isotropic::shared_ptr control_model;// = Isotropic::Sigma(1, 1e-5);    // Dynamics constraints.

	};
}