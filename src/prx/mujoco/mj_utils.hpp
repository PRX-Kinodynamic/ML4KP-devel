#pragma once
#ifndef MUJOCO_NOT_BUILT
#include <vector>
#include "prx/utilities/defs.hpp"

#include "mujoco/mujoco.h"

namespace prx
{
struct mjJointInfo
{
  std::string name;
  int type;
  bool limited;
  mjtNum range[2];
  int qposadr;
  int dofadr;
};

struct mjActuatorInfo
{
  std::string name;
  bool limited;
  mjtNum range[2];
};

std::vector<int> get_qpos_indices(mjModel* m, const std::vector<std::string>& joint_names);

void forward_kinematics(mjModel* m, mjData* d, const std::vector<int>& qpos_inds, 
const std::string& query_link_name, const std::vector<double>& q);

void get_mj_joint_info(mjModel* m, std::vector<mjJointInfo*>& joint_info);
std::ostream& operator<<(std::ostream& os, const mjJointInfo& info);

void get_mj_actuator_info(mjModel* m, std::vector<mjActuatorInfo*>& actuator_info);
std::ostream& operator<<(std::ostream& os, const mjActuatorInfo& info);

struct MujocoState
{
  // Check this issue: https://github.com/deepmind/mujoco/issues/270
  // and this: https://github.com/deepmind/mujoco/issues/493
  // and this: https://github.com/deepmind/dm_control/issues/64
  mjtNum time;
  std::vector<mjtNum> qpos;
  std::vector<mjtNum> qvel;
  std::vector<mjtNum> act;
  std::vector<mjtNum> ctrl;
};
std::ostream& operator<<(std::ostream& os, const MujocoState& state);
}  // namespace prx
#endif