#pragma once

#include "prx/utilities/defs.hpp"
#include "prx/simulation/simulator.hpp"
#include "prx/simulation/playback/trajectory.hpp"

#include "SharedMemory/b3RobotSimulatorClientAPI_InternalData.h"
#include "RobotSimulator/b3RobotSimulatorClientAPI.h"
#include "Bullet3Common/b3HashMap.h"
#include "Bullet3Common/b3Vector3.h"
#include "Bullet3Common/b3Quaternion.h"
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "btBulletDynamicsCommon.h"
#include "LinearMath/btDefaultMotionState.h"
#include "SharedMemory/RemoteGUIHelper.h"


namespace prx
{
	class bullet_simulator_t : public simulator_t
	{
	public:
		bullet_simulator_t(plant_type plants_type, const std::vector<system_ptr_t>& sys_group);
		~bullet_simulator_t();

		void visualize_trajectories(const std::vector<trajectory_t> trajs);

		void print_trajectories(const std::vector<trajectory_t> trajs);

		void visualize_goal(const space_point_t goal, const double radius);

		virtual void step_simulation(propagate_step step) override final;

	private:
		b3RobotSimulatorClientAPI* sim;
		b3RobotSimulatorSetPhysicsEngineParameters physicsArgs;
		b3RobotSimulatorAddUserDebugLineArgs* lineArgs;

		// plant_type sim_type;
		// std::vector<system_ptr_t> group;
	};
}
