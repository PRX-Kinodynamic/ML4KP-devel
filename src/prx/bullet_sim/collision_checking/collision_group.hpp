#pragma once

#include "prx/bullet_sim/bullet_simulator.hpp"
#include "prx/simulation/collision_checking/collision_group.hpp"

#include <vector>

namespace prx
{
	class bullet_collision_group_t : public collision_group_t
	{
	public:
	  	bullet_collision_group_t(const std::vector<system_ptr_t>& in_plants,const std::vector<std::shared_ptr<movable_object_t>>& in_obstacles={});
		bullet_collision_group_t(const std::shared_ptr<bullet_simulator_t> _simulator);
		~bullet_collision_group_t();

		bool in_collision() override;

		protected:

		std::shared_ptr<bullet_simulator_t> simulator;
		std::vector<std::pair<std::pair<int, int>, std::pair<int, int> > > collision_exclusion_list;
		b3RobotSimulatorGetContactPointsArgs contact_args;
		b3ContactInformation *contactInfo = new b3ContactInformation();	

		bool is_contact_excluded(const b3ContactPointData &contact);
	};
}
