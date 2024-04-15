#pragma once
#ifndef MUJOCO_NOT_BUILT
#include <vector>
#include "prx/utilities/defs.hpp"

#include "prx/mujoco/mj_utils.hpp"

#include "mujoco/mujoco.h"
#include "prx/planning/planners/planner.hpp"
#include "prx/planning/planner_functions/planner_functions.hpp"

namespace prx{

void jacobian_steering(mjModel* m, mjData* d, trajectory_t& traj, Eigen::Vector<double, 7> goal_pose, int body_id, std::vector<int> qpos_inds);

void compute_manipulator_jacobian(mjModel* m, mjData* d, Eigen::Matrix<double, 6, 7>& jac, double* jacp, double* jacr, int body_id, std::vector<int> qpos_inds);

std::vector<double> forward_kinematics(mjModel* m, mjData* d, const std::vector<int>& qpos_inds, 
const std::string& query_link_name, const std::vector<double>& q);

std::vector<double> forward_kinematics(mjModel* m, mjData* d, const std::vector<int>& qpos_inds, 
const std::string& query_link_name, const space_point_t& q);

}
#endif