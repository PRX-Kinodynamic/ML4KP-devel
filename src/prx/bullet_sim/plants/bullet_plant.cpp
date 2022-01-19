#ifndef BULLET_NOT_BUILT

#include "prx/bullet_sim/plants/bullet_plant.hpp"
#include <math.h>

namespace prx
{

	bullet_plant_t::bullet_plant_t(const std::string& path) : plant_t(path)
	{
		/*
		Initialize the state and control spaces for the robot.
		*/
		system_type = plant_type::BULLET;
			
	}

	bullet_plant_t::~bullet_plant_t()
	{
	}	

	void bullet_plant_t::propagate(const double simulation_step, const propagate_step step)
	{
		sim->setTimeStep(simulation_step);
		prx_assert(current_state != nullptr, "current_state not initialized!");
		state_space->copy_to_point(current_state);

		if (step == propagate_step::FIRST_STEP)
		{	
			update_to_bullet(current_state);
		}

		if(!is_collision())
		{
		  sim->stepSimulation();
		  bool save_sim_state = (step == propagate_step::FINAL_STEP);
		  this->update_from_bullet(save_sim_state);
		}
	}

	void bullet_plant_t::compute_control()
	{
		/*
		Needs to be implemented individually for each robot.
		*/
	}

	void bullet_plant_t::purge_saved_states()
	{
		std::cout << "Purging all non inital states..." << std::endl;
		for (int i = 1; i < lastSavedId; i++)
		{
			sim->removeStateFromMemory(i);
		} 
		lastSavedId=0;
		std::cout << "Finished purging." << std::endl;
	}

	void bullet_plant_t::update_to_bullet(const space_point_t& point)
	{
		std::cout << "[update_to_bullet] " << state_space->print_point(point,8) << std::endl;
		// @aravind: I think there should be an easier way to do this.
		std::vector<double> current_state_vec;
		state_space->copy_vector_from_point(current_state_vec,point);
		sim->restoreStateFromMemory(current_state_vec.back());

	}

	void bullet_plant_t::update_configuration()
	{
		space_point_t c_state = state_space->make_point();
		state_space->copy_to_point(c_state);
		std::vector<double> current_state_vec;
		state_space->copy_vector_from_point(current_state_vec,c_state);
		sim->restoreStateFromMemory(current_state_vec.back());		
	}

	void bullet_plant_t::compute_derivative()
	{

	}

	void bullet_plant_t::add_exclusion(int bID1, int lID1, int bID2, int lID2)
	{
		std::pair<std::pair<int, int>, std::pair<int,int> > exclusion;
		exclusion.first.first = bID1;
		exclusion.first.second = lID1;
		exclusion.second.first = bID2;
		exclusion.second.second = lID2;
		m_CD_exclusion_list.push_back(exclusion);
  }
  
	bool bullet_plant_t::b_exclude(std::pair<std::pair<int, int>, std::pair<int, int> > excluded_pair, const b3ContactPointData &contact)
	{
		if(excluded_pair.first.first == contact.m_bodyUniqueIdA &&(excluded_pair.first.second == contact.m_linkIndexA || excluded_pair.first.second == -1))
		{
			if(excluded_pair.second.first == contact.m_bodyUniqueIdB &&(excluded_pair.second.second == contact.m_linkIndexB || excluded_pair.second.second == -1))
			{
				return true;
			}
		}
		if(excluded_pair.second.first == contact.m_bodyUniqueIdA &&(excluded_pair.second.second == contact.m_linkIndexA || excluded_pair.second.second == -1))
		{
			if(excluded_pair.first.first == contact.m_bodyUniqueIdB &&(excluded_pair.first.second == contact.m_linkIndexB || excluded_pair.first.second == -1))
			{
				return true;
			}
		}
		return false;
	}

	bool bullet_plant_t::b_exclude_contact(const b3ContactPointData &contact)
	{
		for(int i=0; i < m_CD_exclusion_list.size(); i++)
		{		  
			if(b_exclude(m_CD_exclusion_list[i], contact))
			{
				return true; 
			}
		}  
		return false;
	}

  
	bool bullet_plant_t::is_collision(bool b_include_bounding_box)
	{
		// const space_point_t s;
		b3RobotSimulatorGetContactPointsArgs args;
		b3ContactInformation *contactInfo = new b3ContactInformation();	  
		sim->getContactPoints(args, contactInfo);
		for(int i=0; i<contactInfo->m_numContactPoints; i++)
		{  
			bool b_excluded = b_exclude_contact(contactInfo->m_contactPointData[i]);
			if(!b_excluded)
			{
				return true;
			}
		}
		return false;
	}
  


  	void bullet_plant_t::setBasePositionAndRotation(btVector3 basePosition, btVector3 baseRotation)
  	{
		btQuaternion baseOrientation;
		bullet_simulator_t::get_quaternion_from_euler(baseOrientation, baseRotation);
		sim->resetBasePositionAndOrientation(uniqueId,basePosition,baseOrientation);
  	}
}
#endif
