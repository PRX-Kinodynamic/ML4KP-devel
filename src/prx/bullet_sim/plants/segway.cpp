#include "prx/bullet_sim/plants/segway.hpp"

namespace prx
{
  	segway_t::segway_t(const std::string& path)
  		: bullet_plant_t(path)
  	{
		x = y = yaw = dx = dy = dyaw = sid = 0;

		state_memory = {&x,&y,&yaw,&dx,&dy,&dyaw,&sid};
        state_space = new space_t("EEREEEI",state_memory,"XYZRPYdxdydzdrdpdyaId");
        state_bounds_l = {-14,-14,-3.15,-20,-20,-20,0};
        state_bounds_u = {14,14,3.15,20,20,20,PRX_INFINITY};
        state_space->set_bounds(state_bounds_l, state_bounds_u);

		lf=rf=lr=rr=0;
		if(sync_lr_wheels)
		{
		  control_memory = {&lf,&rf};
		  input_control_space = new space_t("EE",control_memory,"LR");
		  control_bounds_l = {-5,-5};
		  control_bounds_u = {20,20};
		} 
		else
		{
		  control_memory = {&lf,&rf,&lr,&rr};
		  input_control_space = new space_t("EEEE",control_memory,"LfRfLrRr");
		  control_bounds_l = {-5,-5,-5,-5};
		  control_bounds_u = {20,20,20,20};
		}
        input_control_space->set_bounds(control_bounds_l,control_bounds_u); 

		// @aravind: Will this work if it is moved to the bullet plant constructor?
        current_control = input_control_space->make_point();
        current_state = state_space->make_point();
  	}

    void segway_t::reset()
    {
        btVector3 basePosition, baseRotation;
        btQuaternion baseOrientation;

        basePosition[0] = state_space -> at(0);
        basePosition[1] = state_space -> at(1);
		basePosition[2] = 0.2;
		baseRotation[0] = 0;
		baseRotation[1] = 0;
        baseRotation[2] = state_space -> at(2);
        bullet_simulator_t::get_quaternion_from_euler(baseOrientation, baseRotation);

		loadURDFArgs.m_startPosition = basePosition;
        loadURDFArgs.m_startOrientation = baseOrientation;
        uniqueId = sim -> loadURDF(robot_model_path, loadURDFArgs);

		int numJoints = sim->getNumJoints(uniqueId);
		wheelJoints.clear();
		for (int i = 0; i < numJoints; i++)
		{
	  		b3JointInfo jointInfo;
	  		sim->getJointInfo(uniqueId,i,&jointInfo);
	  		
	  		if (jointInfo.m_jointType == 0)
	  		{
				wheelJoints.push_back(i);
	  		}

			b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_VELOCITY);
	  		controlArgs.m_targetVelocity = 0;
	  		controlArgs.m_maxTorqueValue = 0;
	  		sim->setJointMotorControl(uniqueId,i,controlArgs);
		}
        
        for (int i =0; i < 100; i++)
        {
            sim->stepSimulation();
        }
    }

	void segway_t::initialize(std::shared_ptr<b3RobotSimulatorClientAPI> _sim)
    {
        sim = _sim;
        reset();
    }

	segway_t::~segway_t()
	{}

	void segway_t::compute_control()
	{
		input_control_space->copy_to_point(current_control);
		
		b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_VELOCITY);
		controlArgs.m_maxTorqueValue = maxForce;

		if(sync_lr_wheels)
		{
	  		double control_0=controlMultiplier * current_control->at(0);
	  		double control_1=controlMultiplier * current_control->at(1);
			controlArgs.m_targetVelocity = control_0;
	  		sim->setJointMotorControl(uniqueId,wheelJoints[0],controlArgs);
	  		sim->setJointMotorControl(uniqueId,wheelJoints[2],controlArgs);
			
			controlArgs.m_targetVelocity = control_1;
	  		sim->setJointMotorControl(uniqueId,wheelJoints[1],controlArgs);
	  		sim->setJointMotorControl(uniqueId,wheelJoints[3],controlArgs);

		}
		else
		{
	  		for (int i = 0; i < wheelJoints.size(); i++)
			{
				controlArgs.m_targetVelocity = controlMultiplier * current_control->at(i);
		  		sim->setJointMotorControl(uniqueId,wheelJoints[i],controlArgs);
			}
		}
	}
	
	void segway_t::update_from_bullet(const bool save_sim_state)
	{
		current_state_vec.clear();
		btVector3 basePosition, baseRotation;
		btQuaternion baseOrientation;

		sim->getBasePositionAndOrientation(uniqueId,basePosition,baseOrientation);
		bullet_simulator_t::get_euler_from_quaternion(baseRotation, baseOrientation);

		current_state_vec.push_back(basePosition[0]);
		current_state_vec.push_back(basePosition[1]);
		current_state_vec.push_back(baseRotation[0]);
		
		btVector3 baseVel, baseAngVel;
		sim->getBaseVelocity(uniqueId,baseVel,baseAngVel);
		current_state_vec.push_back(baseVel[0]);
		current_state_vec.push_back(baseVel[1]);
		current_state_vec.push_back(baseAngVel[2]);
	
		int sid = get_state_id();
	   
		if (save_sim_state)
		{
			sid = sim->saveStateToMemory();
		} 
		current_state_vec.push_back(sid);
		lastSavedId = std::max(lastSavedId, sid);
		state_space->copy_from_vector(current_state_vec);
        state_space->copy_point_from_vector(current_state,current_state_vec);
	}
}
