#pragma once
#ifndef MUJOCO_NOT_BUILT
#include <vector>
#include "prx/utilities/defs.hpp"

#include "prx/mujoco/mj_utils.hpp"

#include "mujoco/mujoco.h"
#include "prx/planning/planners/planner.hpp"
#include "prx/planning/planner_functions/planner_functions.hpp"

namespace prx{

typedef Eigen::Matrix<double, 6, 7> jacobian_t;

void jacobian_steering(mjModel* m, mjData* d, trajectory_t& traj, Eigen::Vector<double, 7> goal_pose, int body_id, std::vector<int> qpos_inds);

void compute_jacobian(mjModel* m, mjData* d, jacobian_t& jac, double* jacp, double* jacr, int body_id, std::vector<int> qpos_inds);

Eigen::VectorXd  forward_kinematics(mjModel* m, mjData* d, const std::vector<int>& qpos_inds, 
const std::string& query_link_name, const Eigen::VectorXd & q);

Eigen::VectorXd  forward_kinematics(mjModel* m, mjData* d, const std::vector<int>& qpos_inds, 
const std::string& query_link_name, const space_point_t& q);

}
#endif