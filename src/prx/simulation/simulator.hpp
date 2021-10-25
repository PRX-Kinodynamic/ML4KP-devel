#pragma once

#include "prx/utilities/defs.hpp"
#include "prx/simulation/system.hpp"

namespace prx
{
	class simulator_t
	{
	public:
		// simulator_t(plant_type plants_type);
		simulator_t();
		~simulator_t();

		void set_group(const std::vector<system_ptr_t>& sys_group);

		virtual void step_simulation(propagate_step step);

		const plant_type get_simulator_type();

	protected:
		plant_type sim_type;
		std::vector<system_ptr_t> group;
	};
}
