#pragma once


#include "prx/bullet_sim/bullet_defs.hpp"
#include "prx/bullet_sim/plants/bullet_plant.hpp"
#include "prx/bullet_sim/collision_checking/collision_group.hpp"
#include "prx/simulation/collision_checking/collision_checker.hpp"


namespace prx
{

	class bullet_collision_checker_t : public collision_checker_t
	{
	public:
		bullet_collision_checker_t(const std::shared_ptr<bullet_simulator_t> _simulator) : collision_checker_t()
		{
			simulator = _simulator;
		};
		virtual ~bullet_collision_checker_t(){};

		// QUESTION: Overriding function, 2 & 3 params not use... Is there a better alternative?
		void add_collision_group(const std::string& group_name, 
								 const std::vector<system_ptr_t>& in_plants,
								 const std::vector<std::shared_ptr<movable_object_t>>& in_obstacles) override
		{
			collision_groups[group_name] = std::make_shared<bullet_collision_group_t>(simulator);
		}
	
	protected:
		std::shared_ptr<bullet_simulator_t> simulator;

	};
}
