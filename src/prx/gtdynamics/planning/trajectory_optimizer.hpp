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
#include "prx/simulation/system_group.hpp"
#include "prx/simulation/playback/plan.hpp"
#include "prx/utilities/general/zipped_iter.hpp"
#include "prx/simulation/playback/trajectory.hpp"

#include "prx/gtdynamics/defs.hpp"
#include "prx/gtdynamics/factors/factors.hpp"
#include "prx/gtdynamics/utilities/prx_symbols.hpp"
#include "prx/gtdynamics/planning/trajectory_fg.hpp"
#include "prx/gtdynamics/utilities/symbols_factory.hpp"
#include "prx/gtdynamics/planning/initialization_trajs_fg.hpp"
namespace prx
{
	struct traj_opt_params_t
	{
		double traj_rate = 0.5;

	};

	class trajectory_optimizer
	{
		public:
			trajectory_optimizer(const trajectory_t _traj, system_ptr_t _plant, std::shared_ptr<system_group_t> _sys_group, std::string logger = "")
				: traj_original(_traj),
				  logger(out_path + "traj_opt_" + logger + ".txt", ' ', "-"),
				  traj_internal(_sys_group -> get_state_space()),
				  plan(_sys_group -> get_control_space())
			{
				std::cout << "traj_original size: " << traj_original.size() << std::endl;
				sys_group = _sys_group;
				original_rate = 1.0;
				plant = _plant;
				use_input_plan = false;
			}

			// trajectory_optimizer(const trajectory_t _traj, const plan_t , system_ptr_t _plant, std::shared_ptr<system_group_t> _sys_group, std::string logger = "")
			// 	: traj_original(_traj),
			// 	  logger(out_path + "traj_opt_" + logger + ".txt", ' ', "-"),
			// 	  traj_internal(_sys_group -> get_state_space()),
			// 	  plan(_sys_group -> get_control_space()), plan_input(_plan)
			// {
			// 	std::cout << "traj_original size: " << traj_original.size() << std::endl;
			// 	sys_group = _sys_group;
			// 	original_rate = 1.0;
			// 	plant = _plant;
			// 	use_input_plan = true;
			// }

			void optimize(trajectory_t& traj_out, plan_t& plan_out);


			traj_opt_params_t traj_opt_params;
		protected:
			void sample_trajectory();
			void init_plan();
			void sample_trajectory_and_plan();


			trajectory_t traj_original;
			trajectory_t traj_internal;
			
			plan_t plan;
			// plan_input;

			fg_logger_t logger;
			double original_rate;
			system_ptr_t plant;
			bool use_input_plan;

			std::shared_ptr<system_group_t> sys_group;
	};

}
