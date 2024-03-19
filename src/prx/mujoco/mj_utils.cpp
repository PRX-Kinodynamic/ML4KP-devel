#include "prx/mujoco/mj_utils.hpp"

namespace prx
{

std::vector<int> get_qpos_indices(mjModel* m, const mjtObj& obj, const std::vector<std::string>& joint_names)
{
  std::vector<int> qpos_inds(joint_names.size());

  for (int i = 0; i < joint_names.size(); i++){
    int temp_id = mj_name2id(m, obj, joint_names[i].c_str()); // NOTE: mjOBJ_JOINT is hard-coded!
    // qpos_inds.push_back(m->jnt_qposadr[temp_id]);
    if (temp_id == -1)
    {
      prx_throw("Invalid obj, joint_name pair given.")
    }
    qpos_inds[i] = m->jnt_qposadr[temp_id];
  }
  return qpos_inds;
}

std::vector<int> get_body_indices(mjModel* m, const std::string& body_name)
{
  int query_link_id = mj_name2id(m, mjOBJ_BODY, body_name.c_str());

  if (query_link_id == -1)
  {
    prx_throw("Invalid body_name given.")
  }

  std::vector<int> xpos_inds{3*query_link_id, 3*query_link_id+1, 3*query_link_id+2};
  std::vector<int> xquat_inds{4*query_link_id, 4*query_link_id+1, 4*query_link_id+2, 4*query_link_id+3};

  xpos_inds.insert( xpos_inds.end(), xquat_inds.begin(), xquat_inds.end() );
  
  return xpos_inds;
}

std::vector<double> forward_kinematics(mjModel* m, mjData* d, const std::vector<int>& qpos_inds, 
const std::string& query_link_name, const std::vector<double>& q)
{
  prx_assert(qpos_inds.size() == q.size(), "Incorrect configuration length provided.")
  
  // Save current joint qpos values
  double curr_qpos_vals[qpos_inds.size()] = {};
  for(int i = 0; i < qpos_inds.size(); i++){
    curr_qpos_vals[i] = d->qpos[qpos_inds[i]];
    std::cout << curr_qpos_vals[i] << std::endl;
  }

  // Set joint qpos values to those specified by q
  for(int i = 0; i < qpos_inds.size(); i++){
    d->qpos[qpos_inds[i]] = q[i];
  }

  // Call MuJoCo's forward kinematics function 
  // This will output the desired pos, quat values in xpos, xquat
  mj_kinematics(m, d);

  auto body_inds = get_body_indices(m, query_link_name);

  // Retrieve tip link position
  std::vector<double> body_pose(body_inds.size());
  for(int i = 0; i < body_inds.size(); i++){
    if (i <= 2){
      body_pose[i] = d->xpos[body_inds[i]];
    }
    else{
      body_pose[i] = d->xquat[body_inds[i]];
    }
  }

  // Reset joint qpos values to the initial values
  for(int i = 0; i < qpos_inds.size(); i++){
    d->qpos[qpos_inds[i]] = curr_qpos_vals[i];
  }

  // Call mj_kinematics to reset variables
  mj_kinematics(m, d); // is this necessary?

  return body_pose;
}

std::vector<double> forward_kinematics(mjModel* m, mjData* d, const std::vector<int>& qpos_inds, 
const std::string& query_link_name, const space_point_t& q)
{
  prx_assert(qpos_inds.size() == q->get_dim(), "Incorrect configuration length provided.")
  
  std::vector<double> q_vec(q->get_dim());

  for (int i  = 0; i < q_vec.size(); i++){
    q_vec[i] = q->at(i);
  }

  return forward_kinematics(m, d, qpos_inds, query_link_name, q_vec);
}

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
