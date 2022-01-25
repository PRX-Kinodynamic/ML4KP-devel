#pragma once

#include "prx/utilities/defs.hpp"
#include "prx/simulation/simulator.hpp"
#include "prx/simulation/playback/trajectory.hpp"
#include "prx/bullet_sim/plants/bullet_plant.hpp"
#include "prx/simulation/collision_checking/collision_group.hpp"

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
	class bullet_plant_t;
	typedef std::shared_ptr<bullet_plant_t> bullet_plant_ptr_t;
	typedef std::shared_ptr<collision_group_t> collision_group_ptr_t;

	class bullet_simulator_t : public simulator_t
	{
	public:
		bullet_simulator_t();
		~bullet_simulator_t();

		void initialize_simulation();

		void add_urdf(std::string urdf_path,bool collision_contact=false);

		void visualize_trajectories(const std::vector<trajectory_t> trajs);

		void print_trajectories(const std::vector<trajectory_t> trajs);

		void visualize_goal(const space_point_t goal, const double radius);

		virtual void step_simulation(propagate_step step) override final;

		static
		void get_euler_from_quaternion(btVector3& rpy2, const btQuaternion& quat);

		static
	    void get_quaternion_from_euler(btQuaternion& quat, const btVector3& rollPitchYaw);
	  	
		void execute_traj(bullet_plant_ptr_t sys, trajectory_t traj);

		void set_collision_group(collision_group_ptr_t cg_);

		std::shared_ptr<b3RobotSimulatorClientAPI> sim;

		std::vector<int> allowed_collisions;
		std::vector<int> robot_ids;
	private:
		b3RobotSimulatorSetPhysicsEngineParameters physicsArgs;
		b3RobotSimulatorAddUserDebugLineArgs* lineArgs;
		
		std::vector<std::pair<std::string,bool>> urdf_paths;
		collision_group_ptr_t cg;
	};
}
