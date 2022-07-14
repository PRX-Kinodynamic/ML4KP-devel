#pragma once

#include "prx/utilities/defs.hpp"
#include "prx/simulation/system.hpp"
// #include "prx/simulation/simulator.hpp"
#include "prx/simulation/playback/plan.hpp"
#include "prx/simulation/playback/trajectory.hpp"
#include "prx/simulation/collision_checking/collision_checker.hpp"

namespace prx
{
	class simulator_t;
	class system_group_manager_t;

	class system_group_t
	{
	public:
		// system_group_t(const std::vector<system_ptr_t>& sys_group);
		system_group_t(const std::vector<system_ptr_t>& sys_group, plant_type p_type = plant_type::ANALYTICAL);
		~system_group_t();

		void propagate(space_point_t start_state, const plan_t& plan, space_point_t result);
		
		void propagate(space_point_t start_state, const plan_t& plan, trajectory_t& traj);
		
		void propagate(int steps, space_point_t control = nullptr, trajectory_t* traj = nullptr);


		void compute_stopping_maneuver(space_point_t start_state, double& time);
		inline space_t* get_state_space()
		{
			return state_space;
		}

		inline space_t* get_control_space()
		{
			return control_space;
		}

		void propagate_once(propagate_step step, space_point_t control = nullptr);

	protected:

		std::vector<system_ptr_t> group;
		space_t* state_space;
		space_t* control_space;
		simulator_t* sim;

		friend system_group_manager_t;
	};
}
