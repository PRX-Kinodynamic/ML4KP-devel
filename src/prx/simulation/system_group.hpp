#pragma once

#include "prx/utilities/defs.hpp"
#include "prx/simulation/system.hpp"
#include "prx/simulation/simulator.hpp"
#include "prx/simulation/playback/plan.hpp"
#include "prx/simulation/playback/trajectory.hpp"
#include "prx/simulation/collision_checking/collision_checker.hpp"

namespace prx
{
	class system_group_t
	{
	public:
		// system_group_t(const std::vector<system_ptr_t>& sys_group);
		system_group_t(const std::vector<system_ptr_t>& sys_group, plant_type p_type = plant_type::ANALYTICAL);
		~system_group_t();

		void propagate(space_point_t start_state, const plan_t& plan, space_point_t result);
		void propagate(space_point_t start_state, const plan_t& plan, trajectory_t& traj);
		void compute_stopping_maneuver(space_point_t start_state, std::vector<double>&, std::vector<double>&);
		inline space_t* get_state_space()
		{
			return state_space;
		}

		inline space_t* get_control_space()
		{
			return control_space;
		}

		void propagate_once(space_point_t control, propagate_step step);

		void set_simulator(std::shared_ptr<simulator_t> sim_)
		{
			prx_assert(group.size() > 0,"Group must be set up before simulator!");
			sim = sim_;
			sim->set_group(group);
		}

	protected:

		std::vector<system_ptr_t> group;
		space_t* state_space;
		space_t* control_space;
		std::shared_ptr<simulator_t> sim;

	};
}
