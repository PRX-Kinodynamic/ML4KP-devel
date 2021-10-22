#include "prx/bullet_sim/bullet_simulator.hpp"

namespace prx
{

	bullet_simulator_t::bullet_simulator_t(plant_type plants_type, const std::vector<system_ptr_t>& sys_group) 
		: simulator_t(plants_type, sys_group)
	{
		sim = new b3RobotSimulatorClientAPI();
		lineArgs = new b3RobotSimulatorAddUserDebugLineArgs;

		while(!sim->isConnected())
		{
		  	std::cout<<"waiting for connection"<<std::endl;
		  	sim->connect(eCONNECT_GUI);
		  	// sim->connect(eCONNECT_DIRECT);
		}
		// If connecting to an existing physics server, make sure to uncomment the following line.
		// sim->syncBodies();
		sim->configureDebugVisualizer(COV_ENABLE_GUI, 0);
		sim->configureDebugVisualizer(COV_ENABLE_MOUSE_PICKING,0);
		sim->setTimeOut(10);
	       
		sim->setTimeStep(simulation_step);
		physicsArgs.m_deterministicOverlappingPairs = 1;
		sim->setPhysicsEngineParameter(physicsArgs);

		sim->setGravity(btVector3(0,0,-9.8));

		lineArgs->m_lineWidth = 2.0;
		lineArgs->m_colorRGB[1] = lineArgs->m_colorRGB[2] = 0;	
	}

	bullet_simulator_t::~bullet_simulator_t()
	{
		std::cout << "Disconnecting simulation..." << std::endl;
		//purge_saved_states();
		sim->disconnect();		
		std::cout << "Deleting simulation..." << std::endl;
		delete sim;
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
		for(auto s : group)
		{
			s -> propagate(simulation_step, step);
		}
	    sim->stepSimulation();

		// s -> propagate(simulation_step, step);
	}
	
}
