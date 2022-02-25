#pragma once

#include "prx/utilities/defs.hpp"
#include "prx/simulation/simulator.hpp"
#include "prx/simulation/playback/trajectory.hpp"
#include "prx/bullet_sim/plants/bullet_plant.hpp"
#include "prx/bullet_sim/collision_checking/collision_checker.hpp"
#include "prx/bullet_sim/loaders/obstacle_loader.hpp"

#include "prx/bullet_sim/bullet_includes.hpp"




namespace prx
{
	class bullet_plant_t;
	typedef std::shared_ptr<bullet_plant_t> bullet_plant_ptr_t;
	typedef std::shared_ptr<bullet_collision_group_t> bullet_collision_group_ptr_t;

	// template<
 //    class SGM = system_group_manager_t,
 //    class CC  = bullet_collision_group_t
 //    > 
	class bullet_simulator_t 
		: 	public simulator_t,
			public b3RobotSimulatorClientAPI
			// public std::enable_shared_from_this<bullet_simulator_t>
	{
	public:
		bullet_simulator_t();
		~bullet_simulator_t();

		void initialize_simulation();

		// @Edgar Is bool for collision_contact enough?
		// True -> is_obstacle
		// False -> is robot?
		// What about collisions between robots?
		void add_urdf(std::string urdf_path, bool collision_contact=false);

		void visualize_trajectories(const std::vector<trajectory_t> trajs);

		void print_trajectories(const std::vector<trajectory_t> trajs);

		void visualize_goal(const space_point_t goal, const double radius);

		virtual void step_simulation(propagate_step step) override final;

		virtual void reset_simulation() override final;

		void reset_simulation_with_obstacles(std::string obstacles_file);

		void obstacle_loader(std::string obstacles_file);

		static
		void get_euler_from_quaternion(btVector3& rpy2, const btQuaternion& quat);

		static
	    void get_quaternion_from_euler(btQuaternion& quat, const btVector3& rollPitchYaw);
	  	
		void execute_traj(bullet_plant_ptr_t sys, trajectory_t traj);

		void set_collision_group(bullet_collision_group_ptr_t cg_);

		// std::shared_ptr<b3RobotSimulatorClientAPI> sim;

		
		// QUESTION: Right now, this only steps the simulation for the given duration
		// 			 without changing controls of the robots. Should this be change
		// 			 to accept a plan or controller (viz an output) ?
    	/**
    	 * @brief step the simulation for duration (seconds)
    	 * @details Step the simulation for the given duration, in seconds
    	 * 
    	 * @param duration in seconds
    	 */
		void step_simulation(double duration);


		std::vector<int> allowed_collisions;
		std::vector<int> robot_ids;

		// friend bullet_collision_group_t;
	private:
		b3RobotSimulatorSetPhysicsEngineParameters physicsArgs;
		b3RobotSimulatorAddUserDebugLineArgs* lineArgs;
		
		std::vector<std::pair<std::string,bool>> urdf_paths;
		bullet_collision_group_ptr_t cg;

	};
}
