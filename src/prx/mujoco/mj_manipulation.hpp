#pragma once
#ifndef MUJOCO_NOT_BUILT
#include <vector>
#include "prx/utilities/defs.hpp"

#include "prx/mujoco/mj_utils.hpp"
#include "prx/mujoco/mj_simulator.hpp"

#include "mujoco/mujoco.h"
#include "prx/planning/planners/planner.hpp"
#include "prx/planning/planner_functions/planner_functions.hpp"
#include "prx/planning/planner_functions/manipulation_functions.hpp"

namespace prx{

void steer_test(std::shared_ptr<prx::mujoco_simulator_t> sim, trajectory_t& traj, pose_t goal_pose, int body_id, std::vector<int>& qpos_inds, const config_t& q_init=Eigen::VectorXd());

void jacobian_steering(std::shared_ptr<prx::mujoco_simulator_t> sim, trajectory_t& traj, pose_t goal_pose, int body_id, std::vector<int>& qpos_inds, const config_t& q_init=Eigen::VectorXd());

void compute_jacobian(mjModel* m, mjData* d, jacobian_t& jac, double* jacp, double* jacr, int body_id, std::vector<int>& qpos_inds);

Eigen::VectorXd forward_kinematics(mjModel* m, mjData* d, const std::vector<int>& qpos_inds, 
const std::string& query_link_name, const Eigen::VectorXd & q);

Eigen::VectorXd forward_kinematics(mjModel* m, mjData* d, const std::vector<int>& qpos_inds, 
const std::string& query_link_name, const space_point_t& q);

}
#endif