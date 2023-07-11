// #include "prx/simulation/simulator.hpp"

// namespace prx
// {
// 	simulator_t::simulator_t()
// 	{
// 		sim_type = plant_type::ANALYTICAL;
// 		// group = sys_group;
// 	}

// 	simulator_t::~simulator_t()
// 	{
// 	}

// 	void simulator_t::set_group(const std::vector<system_ptr_t>& sys_group)
// 	{
// 		group = sys_group;
// 	}

// 	void simulator_t::step_simulation(propagate_step step)
// 	{
// 		for(auto s : group)
// 		{
// 			s -> propagate(simulation_step, step);
// 		}
// 	}

// 	const plant_type simulator_t::get_simulator_type()
// 	{
// 		return sim_type;
// 	}

// }
