
#include "prx/bullet_sim/collision_checking/collision_group.hpp"

namespace prx
{
  	bullet_collision_group_t::bullet_collision_group_t(const std::vector<system_ptr_t>& in_plants,const std::vector<std::shared_ptr<movable_object_t>>& in_obstacles)
	{
		prx_warn("You are using a Bullet plant. Please set up the Bullet collision group separately.")
	}

	bullet_collision_group_t::bullet_collision_group_t(const std::shared_ptr<bullet_simulator_t> _simulator)
	{
		simulator = _simulator;

		for (auto i : simulator->allowed_collisions)
		{
			for (auto j : simulator->robot_ids)
			{
				std::cout << "[bullet_collision_group_t] Adding an allowed collision between " << i << " and " << j << std::endl;
				collision_exclusion_list.push_back(std::make_pair(std::make_pair(i, -1), std::make_pair(j, -1)));
			}
		}
	}


	bullet_collision_group_t::~bullet_collision_group_t()
	{
	}

	bool bullet_collision_group_t::is_contact_excluded(const b3ContactPointData &contact)
	{
		for (auto i : collision_exclusion_list)
		{
			if(i.first.first == contact.m_bodyUniqueIdA &&(i.first.second == contact.m_linkIndexA || i.first.second == -1))
			{
				if(i.second.first == contact.m_bodyUniqueIdB &&(i.second.second == contact.m_linkIndexB || i.second.second == -1))
				{
					return true;
				}
			}
			if(i.second.first == contact.m_bodyUniqueIdA &&(i.second.second == contact.m_linkIndexA || i.second.second == -1))
			{
				if(i.first.first == contact.m_bodyUniqueIdB &&(i.first.second == contact.m_linkIndexB || i.first.second == -1))
				{
					return true;
				}
			}
		}
		return false;
	}

	bool bullet_collision_group_t::in_collision()	  
	{	
		// We don't need this since we are using the bullet simulator,
		// but we keep it here for reference.
		update_plants();

		// @aravind:
		// I'm creating the contactInfo object in the class declaration to save time.
		// But, depending on how it is populated inside Bullet, it may not be 
		// getting reset properly. Need to check this.
		simulator->sim->getContactPoints(contact_args, contactInfo);
		for(int i=0; i<contactInfo->m_numContactPoints; i++)
		{  
			bool excluded = is_contact_excluded(contactInfo->m_contactPointData[i]);
			if(!excluded) return true;
		}
	    return false;
	}

}
