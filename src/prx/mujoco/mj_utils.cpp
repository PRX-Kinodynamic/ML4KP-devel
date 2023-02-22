#ifndef MUJOCO_NOT_BUILT
#include "prx/mujoco/mj_utils.hpp"

namespace prx
{
void get_mj_joint_info(mjModel* m, std::vector<mjJointInfo*>& joint_info)
{
  for (int i = 0; i < m->njnt; i++)
  {
    mjJointInfo* info = new mjJointInfo();
    info->name = std::string(m->names + m->name_jntadr[i]);
    info->type = m->jnt_type[i];
    info->limited = (bool)m->jnt_limited[i];
    info->range[0] = m->jnt_range[i * 2];
    info->range[1] = m->jnt_range[i * 2 + 1];
    info->qposadr = m->jnt_qposadr[i];
    info->dofadr = m->jnt_dofadr[i];
    joint_info.push_back(info);
  }
}

std::ostream& operator<<(std::ostream& os, const mjJointInfo& info)
{
  os << "Name: " << info.name << std::endl;
  os << "Type: " << info.type << std::endl;
  os << "Limited: " << info.limited << std::endl;
  os << "Range: " << info.range[0] << ", " << info.range[1] << std::endl;
  os << "Qposadr: " << info.qposadr << std::endl;
  os << "Dofadr: " << info.dofadr << std::endl;
  return os;
}

void get_mj_actuator_info(mjModel* m, std::vector<mjActuatorInfo*>& actuator_info)
{
  for (int i = 0; i < m->nu; i++)
  {
    mjActuatorInfo* info = new mjActuatorInfo();
    info->limited = (bool)m->actuator_ctrllimited[i];
    info->range[0] = m->actuator_ctrlrange[i * 2];
    info->range[1] = m->actuator_ctrlrange[i * 2 + 1];
    actuator_info.push_back(info);
  }
}

std::ostream& operator<<(std::ostream& os, const mjActuatorInfo& info)
{
  os << "Limited: " << info.limited << std::endl;
  os << "Range: " << info.range[0] << ", " << info.range[1] << std::endl;
  return os;
}

std::ostream& operator<<(std::ostream& os, const MujocoState& state)
{
  os << "Time: " << state.time << std::endl;
  os << "Qpos: ";
  for (auto& q : state.qpos)
    os << q << ", ";
  os << std::endl;
  os << "Qvel: ";
  for (auto& q : state.qvel)
    os << q << ", ";
  os << std::endl;
  os << "Act: ";
  for (auto& q : state.act)
    os << q << ", ";
  os << std::endl;
  os << "Ctrl: ";
  for (auto& q : state.ctrl)
    os << q << ", ";
  os << std::endl;
  return os;
}
}  // namespace prx
#endif