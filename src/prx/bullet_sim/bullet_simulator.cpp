#include "prx/bullet_sim/bullet_simulator.hpp"

namespace prx
{

	bullet_simulator_t::bullet_simulator_t() 
		: simulator_t()
	{
		sim_type = plant_type::BULLET; 

		sim = std::make_shared<b3RobotSimulatorClientAPI>();
		// sim = new b3RobotSimulatorClientAPI();
		lineArgs = new b3RobotSimulatorAddUserDebugLineArgs;

		while(!sim->isConnected())
		{
		  	std::cout<<"waiting for connection"<<std::endl;
		  	sim->connect(eCONNECT_GUI);
		  	// sim -> connect(eCONNECT_SHARED_MEMORY);
		  	// sim->connect(eCONNECT_DIRECT);
		}
		// If connecting to an existing physics server, make sure to uncomment the following line.
		sim->syncBodies();
		sim->configureDebugVisualizer(COV_ENABLE_GUI, 0);
		sim->configureDebugVisualizer(COV_ENABLE_MOUSE_PICKING,0);
		sim->setTimeOut(10);
	       
		sim->setTimeStep(simulation_step);
		physicsArgs.m_deterministicOverlappingPairs = 1;
		sim->setPhysicsEngineParameter(physicsArgs);

		sim->setGravity(btVector3(0,0,-9.8));

		lineArgs->m_lineWidth = 2.0;
		lineArgs->m_colorRGB[1] = lineArgs->m_colorRGB[2] = 0;	
	  	
		sim -> setRealTimeSimulation(false);

	}

	bullet_simulator_t::~bullet_simulator_t()
	{
		std::cout << "Disconnecting simulation..." << std::endl;
		//purge_saved_states();
		sim->disconnect();		
		std::cout << "Deleting simulation..." << std::endl;
		// delete sim;
	}

	void bullet_simulator_t::initialize_simulation()
	{
		for (auto f : urdf_paths)
		{
			std::cout << "f: " << f.first << std::endl;
			int body_id = sim -> loadURDF(f.first);
			if (!f.second) allowed_collisions.push_back(body_id);
		}

		for (auto s : group)
		{
			auto sb = std::dynamic_pointer_cast<bullet_plant_t>(s);
			sb -> initialize(sim);
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
		sim->resetDebugVisualizerCamera(4.0,-90.4,180.1,targetPos);		
		sim->restoreStateFromMemory(0);
		for (auto traj : trajs)
		{
  			for (int i = 1; i < traj.size()-1; i++)
			{
				unsigned idx = i;
				double* startLine = new double[3]{traj[idx]->at(0),traj[idx]->at(1),0.2};
				double* endLine = new double[3]{traj[idx+1]->at(0),traj[idx+1]->at(1),0.2};
				sim->addUserDebugLine(startLine,endLine,*lineArgs);
			}
		}
	}

	void bullet_simulator_t::visualize_goal(const space_point_t goal, const double radius)
	{
		btVector3 pos;
		pos[0] = goal->at(0); pos[1] = goal->at(1); pos[2] = 0.2;	
		b3RobotSimulatorAddUserDebugTextArgs* textArgs = new b3RobotSimulatorAddUserDebugTextArgs;
		textArgs->m_colorRGB[0] = textArgs->m_colorRGB[1] = textArgs->m_colorRGB[2] = 0;
		sim->addUserDebugText("GOAL",pos,*textArgs);
	}

	void bullet_simulator_t::step_simulation(propagate_step step)
	{	
		prx_assert(cg != nullptr,"Bullet collision group is NULL!");
		for(auto s : group)
		{
			auto sb = std::dynamic_pointer_cast<bullet_plant_t>(s);
			if (step == propagate_step::FIRST_STEP)
			{	
				sim -> restoreStateFromMemory(sb -> get_state_id());
			}

			if(! cg -> in_collision())
			{
				sim -> stepSimulation();
				bool save_sim_state = (step == propagate_step::FINAL_STEP);
				sb -> update_from_bullet(save_sim_state);
			}
		}
	}
	
	void bullet_simulator_t::execute_traj(bullet_plant_ptr_t sys, trajectory_t traj)
  	{
		btVector3 targetPos;
		targetPos[0] = targetPos[1] = targetPos[2] = 0;
		sim -> resetDebugVisualizerCamera(15.0,-90.4,180.1,targetPos);	
    	sim -> restoreStateFromMemory(0);
    	for(int i=0; i<traj.size(); i++)
    	{
      		usleep(8000);
      		space_point_t point = traj[(unsigned)i];
      		int inpt;
      		sim -> restoreStateFromMemory(sys -> get_state_id());
    	}
	}
}
