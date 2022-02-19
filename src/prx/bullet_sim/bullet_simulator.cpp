#include "prx/bullet_sim/bullet_simulator.hpp"
#include "Utils/b3Clock.h"

namespace prx
{

	bullet_simulator_t::bullet_simulator_t() 
		: simulator_t(plant_type::BULLET)
	{
		// system_groups = std::make_shared<system_group_manager_t>();
		// system_groups -> sim = this -> shared_ptr();

		// system_groups -> sim = this -> shared_ptr();
		// sim = std::make_shared<b3RobotSimulatorClientAPI>();
		// sim = new b3RobotSimulatorClientAPI();
		lineArgs = new b3RobotSimulatorAddUserDebugLineArgs;

		while(!this -> isConnected())
		{
		  	std::cout<<"waiting for connection"<<std::endl;
		  	this ->connect(eCONNECT_GUI);
		  	// sim -> connect(eCONNECT_SHARED_MEMORY);
		  	// sim->connect(eCONNECT_DIRECT);
		}
		// If connecting to an existing physics server, make sure to uncomment the following line.
		this -> syncBodies();
		this -> configureDebugVisualizer(COV_ENABLE_GUI, 0);
		this -> configureDebugVisualizer(COV_ENABLE_MOUSE_PICKING,0);
		this -> setTimeOut(10);
	       
		this -> setTimeStep(simulation_step);
		physicsArgs.m_deterministicOverlappingPairs = 1;
		this -> setPhysicsEngineParameter(physicsArgs);

		this -> setGravity(btVector3(0,0,-9.8));

		lineArgs->m_lineWidth = 2.0;
		lineArgs->m_colorRGB[1] = lineArgs->m_colorRGB[2] = 0;	
	  	
		this -> setRealTimeSimulation(false);

	}

	bullet_simulator_t::~bullet_simulator_t()
	{
		std::cout << "Disconnecting simulation..." << std::endl;
		//purge_saved_states();
		this -> disconnect();		
		std::cout << "Deleting simulation..." << std::endl;
		// delete sim;
	}

	void bullet_simulator_t::initialize_simulation()
	{
		system_groups -> link_simulator(this);

		for (auto f : urdf_paths)
		{
			std::cout << "f: " << f.first << std::endl;
			int body_id = this -> loadURDF(f.first);
			if (!f.second) allowed_collisions.push_back(body_id);
		}

		std::string context_name = "bullet_context";
		std::vector<system_ptr_t> context_systems;
			// std::vector<std::shared_ptr<movable_object_t>> context_obstacles;
		// for(auto&& s : system_names)
		for (auto s_pair : this -> systems)
		{
			auto s = s_pair.second;
			// context_systems.push_back(this -> systems[s]);
			context_systems.push_back(s);

			auto sb = std::dynamic_pointer_cast<bullet_plant_t>(s);
			auto ptr = std::static_pointer_cast<bullet_simulator_t>(this -> shared_ptr());
			sb -> initialize(ptr);
			sb -> update_from_bullet(true);
			robot_ids.push_back(sb->uniqueId);
			// TODO: Add exclusions between the system and the plane.
		}
		system_groups->add_system_group(context_name,context_systems);

		auto ptr = std::static_pointer_cast<bullet_simulator_t>(this -> shared_ptr());
		collision_groups.reset(new bullet_collision_checker_t(ptr));

		collision_groups->add_collision_group(context_name,context_systems,{});

		// this -> collision_groups -> link_simulator(this -> get_ptr());
	}

	void bullet_simulator_t::reset_simulation()
	{
		this -> resetSimulation();
		this -> setGravity(btVector3(0,0,-9.8));

		for (auto f : urdf_paths)
		{
			std::cout << "f: " << f.first << std::endl;
			int body_id = this -> loadURDF(f.first);
			if (!f.second) allowed_collisions.push_back(body_id);
		}

		robot_ids.clear();

		for (auto s_pair : this -> systems)
		{
			auto s = s_pair.second;

			auto sb = std::dynamic_pointer_cast<bullet_plant_t>(s);
			sb -> reset();
			sb -> update_from_bullet(true);
			robot_ids.push_back(sb->uniqueId);
			// TODO: Add exclusions between the system and the plane.
		}
	}

	void bullet_simulator_t::set_collision_group(collision_group_ptr_t cg_)
	{
		cg = cg_;
	}

	void bullet_simulator_t::add_urdf(std::string urdf_path,bool collision_contact)
	{
		// By default, it is NOT a collision contact.
		urdf_paths.push_back(std::make_pair(urdf_path,collision_contact));
	}

	void bullet_simulator_t::get_euler_from_quaternion(btVector3& rpy2, const btQuaternion& quat)
	{
		btScalar roll, pitch, yaw;
		quat.getEulerZYX(yaw, pitch, roll);
		rpy2.setValue(yaw, pitch, roll);
	}

	void bullet_simulator_t::get_quaternion_from_euler(btQuaternion& quat, const btVector3& rollPitchYaw)
	{
		quat.setEulerZYX(rollPitchYaw[2], rollPitchYaw[1], rollPitchYaw[0]);
	}

	void bullet_simulator_t::print_trajectories(const std::vector<trajectory_t> trajs)
	{
		std::cout << "Number of trajectories = " << trajs.size() << std::endl;
		btVector3 targetPos;
		targetPos[0] = targetPos[1] = targetPos[2] = 0;

		int j=0;
		for (auto traj : trajs)
		{
		  	std::cout<<"Trajectory "<<j<<":"<<std::endl;
		  	j++;
  			for (int i = 0; i < traj.size()-1; i++)
			{
				unsigned idx = i;
				//to do - make this depend on dimension
				double* startLine = new double[3]{traj[idx]->at(0),traj[idx]->at(1),traj[idx]->at(2)};
				double* endLine = new double[3]{traj[idx+1]->at(0),traj[idx+1]->at(1),traj[idx]->at(2)};
				if(i==0)
				{
				  	std::cout<<startLine[0]<<", "<<startLine[1]<<", "<<startLine[2]<<std::endl;
				}
				std::cout<<endLine[0]<<", "<<endLine[1]<<", "<<endLine[2]<<std::endl;
			}
		}
	}
  
	void bullet_simulator_t::visualize_trajectories(const std::vector<trajectory_t> trajs)
	{
		std::cout << "Number of trajectories = " << trajs.size() << std::endl;
		btVector3 targetPos;
		targetPos[0] = targetPos[1] = targetPos[2] = 0;
		this -> resetDebugVisualizerCamera(4.0,-90.4,180.1,targetPos);		
		this -> restoreStateFromMemory(0);
		for (auto traj : trajs)
		{
  			for (int i = 1; i < traj.size()-1; i++)
			{
				unsigned idx = i;
				double* startLine = new double[3]{traj[idx]->at(0),traj[idx]->at(1),0.2};
				double* endLine = new double[3]{traj[idx+1]->at(0),traj[idx+1]->at(1),0.2};
				this ->addUserDebugLine(startLine,endLine,*lineArgs);
			}
		}
	}

	void bullet_simulator_t::visualize_goal(const space_point_t goal, const double radius)
	{
		btVector3 pos;
		pos[0] = goal->at(0); pos[1] = goal->at(1); pos[2] = 0.2;	
		b3RobotSimulatorAddUserDebugTextArgs* textArgs = new b3RobotSimulatorAddUserDebugTextArgs;
		textArgs->m_colorRGB[0] = textArgs->m_colorRGB[1] = textArgs->m_colorRGB[2] = 0;
		this ->addUserDebugText("GOAL",pos,*textArgs);
	}

	void bullet_simulator_t::step_simulation(propagate_step step)
	{	
		prx_assert(cg != nullptr,"Bullet collision group is NULL!");
		for(auto s_pair : this -> systems)
		{
			auto s = s_pair.second;
			auto sb = std::dynamic_pointer_cast<bullet_plant_t>(s);
			if (step == propagate_step::FIRST_STEP)
			{	
				this -> restoreStateFromMemory(sb -> get_state_id());
			}

			if(! cg -> in_collision())
			{
				this -> stepSimulation();
				bool save_sim_state = (step == propagate_step::FINAL_STEP);
				sb -> update_from_bullet(save_sim_state);
			}
		}
	}
	
	void bullet_simulator_t::execute_traj(bullet_plant_ptr_t sys, trajectory_t traj)
  	{
		btVector3 targetPos;
		targetPos[0] = targetPos[1] = targetPos[2] = 0;
		this -> resetDebugVisualizerCamera(15.0,-90.4,180.1,targetPos);	
    	this -> restoreStateFromMemory(0);
    	for(int i=0; i<traj.size(); i++)
    	{
      		usleep(8000);
      		space_point_t point = traj[(unsigned)i];
      		int inpt;
      		this -> restoreStateFromMemory(sys -> get_state_id());
    	}
	}


	void bullet_simulator_t::step_simulation(double duration)
	{
		prx_assert(this -> canSubmitCommand(), "Error with bullet simulation: Cannot submit command (bullet_simulator_t::canSubmitCommand)");
		
		int rotateCamera = 0;

		b3KeyboardEventsData keyEvents;
		this -> getKeyboardEvents(&keyEvents);
		if (keyEvents.m_numKeyboardEvents)
		{
			for (int i = 0; i < keyEvents.m_numKeyboardEvents; i++)
			{
				b3KeyboardEvent& e = keyEvents.m_keyboardEvents[i];

				if (e.m_keyCode == 'r' && e.m_keyState & eButtonTriggered)
				{
					rotateCamera = 1 - rotateCamera;
				}

			}
		}
		this -> stepSimulation();

		if (rotateCamera)
		{
			static double yaw = 0;
			double distance = 1;
			yaw += 0.1;
			btVector3 basePos;
			btQuaternion baseOrn;
			// sim->getBasePositionAndOrientation(minitaurUid, basePos, baseOrn);
			this -> resetDebugVisualizerCamera(distance, -20, yaw, basePos);
		}
		const double one_second = 1e+6; // microseconds
		// double microSeconds = one_second * duration;
		// void b3Clock::usleep(int microSeconds)
		b3Clock::usleep(one_second * duration);

	}

}
