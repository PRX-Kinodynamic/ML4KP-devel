#include "prx/bullet_sim/plants/racecar.hpp"

namespace prx
{
	racecar_t::racecar_t(const std::string& path): bullet_plant_t(path)
	{
		x = y = theta = sid = fwd = steer = 0;

		state_memory = {&x, &y, &theta, &sid};
		state_space = new space_t("EERD",state_memory,"XYThetaId");
		state_space->set_bounds({-15,-15,-3.15,0},{15,15,3.15,PRX_INFINITY});

		control_memory = {&fwd,&steer};
		input_control_space = new space_t("ER",control_memory,"FwdSteer");
		input_control_space->set_bounds({-1,-1},{1,1});

		current_control = input_control_space->make_point();
        current_state = state_space->make_point();
  	}

	void racecar_t::initialize(std::shared_ptr<b3RobotSimulatorClientAPI> _sim)
	{
		sim = _sim;
		simulation_step = 0.01;
		sim->setTimeStep(simulation_step);

		btVector3 basePosition, baseRotation;
		btQuaternion baseOrientation;

		basePosition[0] = state_space -> at(0);
		basePosition[1] = state_space -> at(1);
		basePosition[2] = 0.2;
		baseRotation[2] = state_space -> at(2);
        bullet_simulator_t::get_quaternion_from_euler(baseOrientation, baseRotation);

		loadURDFArgs.m_startPosition = basePosition;
        loadURDFArgs.m_startOrientation = baseOrientation;
        uniqueId = sim -> loadURDF(robot_model_path, loadURDFArgs);

		int numJoints = sim->getNumJoints(uniqueId);
		for (int i = 0; i < numJoints; i++)
		{
			b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_VELOCITY);
			controlArgs.m_targetVelocity = 0;
			controlArgs.m_maxTorqueValue = 0;
			sim->setJointMotorControl(uniqueId,i,controlArgs);
		}


		b3JointInfo* jointInfo = new b3JointInfo;
		b3RobotUserConstraint* constraintInfo = new b3RobotUserConstraint;

		jointInfo->m_jointType = (int) JointType::eGearType;
		jointInfo->m_jointAxis[0] = 0; jointInfo->m_jointAxis[1] = 0; jointInfo->m_jointAxis[2] = 0;;
		for (int i = 0; i < 3; i++) jointInfo->m_parentFrame[i] = 0;
		for (int i = 0; i < 3; i++) jointInfo->m_childFrame[i] = 0;
		int constraintId = sim->createConstraint(uniqueId,9,uniqueId,11,jointInfo);
		constraintInfo->setGearRatio(1);
		constraintInfo->setMaxAppliedForce(10000);
		sim->changeConstraint(constraintId,constraintInfo);

		jointInfo->m_jointType = (int) JointType::eGearType;
		jointInfo->m_jointAxis[0] = 0; jointInfo->m_jointAxis[1] = 0; jointInfo->m_jointAxis[2] = 0;;
		for (int i = 0; i < 3; i++) jointInfo->m_parentFrame[i] = 0;
		for (int i = 0; i < 3; i++) jointInfo->m_childFrame[i] = 0;
		constraintId = sim->createConstraint(uniqueId,10,uniqueId,13,jointInfo);
		constraintInfo->setGearRatio(-1);
		constraintInfo->setMaxAppliedForce(10000);
		sim->changeConstraint(constraintId,constraintInfo);

		jointInfo->m_jointType = (int) JointType::eGearType;
		jointInfo->m_jointAxis[0] = 0; jointInfo->m_jointAxis[1] = 0; jointInfo->m_jointAxis[2] = 0;;
		for (int i = 0; i < 3; i++) jointInfo->m_parentFrame[i] = 0;
		for (int i = 0; i < 3; i++) jointInfo->m_childFrame[i] = 0;
		constraintId = sim->createConstraint(uniqueId,9,uniqueId,13,jointInfo);
		constraintInfo->setGearRatio(-1);
		constraintInfo->setMaxAppliedForce(10000);
		sim->changeConstraint(constraintId,constraintInfo);

		jointInfo->m_jointType = (int) JointType::eGearType;
		jointInfo->m_jointAxis[0] = 0; jointInfo->m_jointAxis[1] = 0; jointInfo->m_jointAxis[2] = 0;;
		for (int i = 0; i < 3; i++) jointInfo->m_parentFrame[i] = 0;
		for (int i = 0; i < 3; i++) jointInfo->m_childFrame[i] = 0;
		constraintId = sim->createConstraint(uniqueId,16,uniqueId,18,jointInfo);
		constraintInfo->setGearRatio(1);
		constraintInfo->setMaxAppliedForce(10000);
		sim->changeConstraint(constraintId,constraintInfo);

		jointInfo->m_jointType = (int) JointType::eGearType;
		jointInfo->m_jointAxis[0] = 0; jointInfo->m_jointAxis[1] = 0; jointInfo->m_jointAxis[2] = 0;;
		for (int i = 0; i < 3; i++) jointInfo->m_parentFrame[i] = 0;
		for (int i = 0; i < 3; i++) jointInfo->m_childFrame[i] = 0;
		constraintId = sim->createConstraint(uniqueId,16,uniqueId,19,jointInfo);
		constraintInfo->setGearRatio(-1);
		constraintInfo->setMaxAppliedForce(10000);
		sim->changeConstraint(constraintId,constraintInfo);

		jointInfo->m_jointType = (int) JointType::eGearType;
		jointInfo->m_jointAxis[0] = 0; jointInfo->m_jointAxis[1] = 0; jointInfo->m_jointAxis[2] = 0;;
		for (int i = 0; i < 3; i++) jointInfo->m_parentFrame[i] = 0;
		for (int i = 0; i < 3; i++) jointInfo->m_childFrame[i] = 0;
		constraintId = sim->createConstraint(uniqueId,17,uniqueId,19,jointInfo);
		constraintInfo->setGearRatio(-1);
		constraintInfo->setMaxAppliedForce(10000);
		sim->changeConstraint(constraintId,constraintInfo);

		jointInfo->m_jointType = (int) JointType::eGearType;
		jointInfo->m_jointAxis[0] = 0; jointInfo->m_jointAxis[1] = 0; jointInfo->m_jointAxis[2] = 0;;
		for (int i = 0; i < 3; i++) jointInfo->m_parentFrame[i] = 0;
		for (int i = 0; i < 3; i++) jointInfo->m_childFrame[i] = 0;
		constraintId = sim->createConstraint(uniqueId,1,uniqueId,18,jointInfo);
		constraintInfo->setGearRatio(-1);
		constraintInfo->setGearAuxLink(15);
		constraintInfo->setMaxAppliedForce(10000);
		sim->changeConstraint(constraintId,constraintInfo);

		jointInfo->m_jointType = (int) JointType::eGearType;
		jointInfo->m_jointAxis[0] = 0; jointInfo->m_jointAxis[1] = 0; jointInfo->m_jointAxis[2] = 0;;
		for (int i = 0; i < 3; i++) jointInfo->m_parentFrame[i] = 0;
		for (int i = 0; i < 3; i++) jointInfo->m_childFrame[i] = 0;
		constraintId = sim->createConstraint(uniqueId,3,uniqueId,19,jointInfo);
		constraintInfo->setGearRatio(-1);
		constraintInfo->setGearAuxLink(15);
		constraintInfo->setMaxAppliedForce(10000);
		sim->changeConstraint(constraintId,constraintInfo);
  
		for (int i =0; i < 100; i++)
		{
			sim->stepSimulation();
		}

		add_exclusion(0,-1,1,-1);  //exclude collisions with plane
	}
	
	racecar_t::~racecar_t()
	{}

	void racecar_t::compute_control()
	{
		input_control_space->copy_to_point(current_control);

		double targetVelocity = current_control->at(0) * speedMultiplier;
		double steeringAngle  = current_control->at(1) * steeringMultiplier;

		b3RobotSimulatorJointMotorArgs controlArgs_speed(CONTROL_MODE_VELOCITY);
		controlArgs_speed.m_maxTorqueValue = maxForce;
		controlArgs_speed.m_targetVelocity = targetVelocity;

		for (int i = 0; i < motorizedWheels.size(); i++)
		{
			sim->setJointMotorControl(uniqueId,motorizedWheels[i],controlArgs_speed);
		}
		b3RobotSimulatorJointMotorArgs controlArgs_steering(CONTROL_MODE_POSITION_VELOCITY_PD);
		controlArgs_steering.m_targetPosition = steeringAngle;

		for (int i = 0; i < steeringLinks.size(); i++)
		{
			sim->setJointMotorControl(uniqueId,steeringLinks[i],controlArgs_steering);
		}
	}

	void racecar_t::update_from_bullet(const bool save_sim_state)
	{
		std::vector<double> current_state_vec;
		btVector3 basePosition, baseRotation;
		btQuaternion baseOrientation;
		sim->getBasePositionAndOrientation(uniqueId,basePosition,baseOrientation);

		bullet_simulator_t::get_euler_from_quaternion(baseRotation, baseOrientation);
		current_state_vec.push_back(basePosition[0]);
		current_state_vec.push_back(basePosition[1]);
		current_state_vec.push_back(baseRotation[2]);
		int sid = state_space->at(3);
		if (save_sim_state) sid = sim->saveStateToMemory();

		current_state_vec.push_back(sid);
		lastSavedId = std::max(lastSavedId, sid);
		state_space -> copy_from_vector(current_state_vec);
        state_space->copy_point_from_vector(current_state,current_state_vec);
	}
}


