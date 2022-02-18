#pragma once
#include <unistd.h>

#include "prx/bullet_sim/bullet_defs.hpp"

#include "prx/simulation/plant.hpp"
#include "prx/bullet_sim/bullet_simulator.hpp"
#include "prx/simulation/playback/trajectory.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"

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
	typedef std::shared_ptr<bullet_plant_t> bullet_ptr_t;

	class bullet_plant_t : public plant_t
	{
	public:
		bullet_plant_t(const std::string& path);
		virtual ~bullet_plant_t();
		
		virtual void initialize(std::shared_ptr<b3RobotSimulatorClientAPI> sim) = 0;

		virtual void propagate(const double simulation_step, const propagate_step step) override final;

		virtual void compute_control() override;

		virtual void update_to_bullet(const space_point_t& point);

		virtual void update_configuration() override final;

		virtual void purge_saved_states();

		virtual void reset() = 0;

		/**
		 * @brief      Gets the state identifier - Within Bullet, each saved state has an unique id.
		 *
		 * @return     The state identifier.
		 */
		virtual int get_state_id()
		{
			return state_space->at(state_space->get_dimension()-1);
		}

        virtual void update_from_bullet(const bool save_sim_state)
        {
        	prx_throw("Not implemented");
        }

		void setBasePositionAndRotation(btVector3 basePosition, btVector3 baseRotation);

		std::vector<double> state_bounds_l, state_bounds_u, control_bounds_l, control_bounds_u;

		int uniqueId;

		b3RobotSimulatorAddUserDebugLineArgs* lineArgs = new b3RobotSimulatorAddUserDebugLineArgs;

	protected:
		std::vector<double> state_vec, control_vec;
		std::shared_ptr<b3RobotSimulatorClientAPI> sim;

		b3RobotSimulatorSetPhysicsEngineParameters physicsArgs;
        b3RobotSimulatorLoadUrdfFileArgs loadURDFArgs;

		int physicsClientId, lastSavedId;

		std::vector<std::pair<std::pair<int, int>, std::pair<int, int> > > m_CD_exclusion_list;

		virtual void compute_derivative() override final;

		std::string control_topo;
		std::string state_topo;

		space_point_t current_state;
		space_point_t current_control;
        std::vector<double> current_state_vec;
		// friend bullet_simulator;
	};

	
}
