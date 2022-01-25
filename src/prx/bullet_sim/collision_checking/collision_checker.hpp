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
		bullet_collision_checker_t() : collision_checker_t(){};
		virtual ~bullet_collision_checker_t(){};
	};
}
