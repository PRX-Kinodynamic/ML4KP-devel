#include "prx/mujoco/mj_utils.hpp"

namespace prx
{

std::vector<int> get_qpos_indices(mjModel* m, const mjtObj& obj, const std::string& name)
{
  prx_assert(obj == mjOBJ_BODY || obj == mjOBJ_JOINT, "Invalud mjObj type. Only mjOBJ_BODY, mjOBJ_JOINT are currently accepted");
  std::vector<int> qpos_inds{};
  
  if (obj == mjOBJ_BODY){
    qpos_inds = get_body_qpos_indices(m, name);
  }
  else if (obj == mjOBJ_JOINT){
    qpos_inds = get_joint_qpos_indices(m, name);
  }

  return qpos_inds;
}

std::vector<int> get_qpos_indices(mjModel* m, const mjtObj& obj, const std::vector<std::string>& names)
{
  prx_assert(obj == mjOBJ_BODY || obj == mjOBJ_JOINT, "Invalud mjObj type. Only mjOBJ_BODY, mjOBJ_JOINT are currently accepted");
  std::vector<int> qpos_inds{};
  
  std::vector<int> temp_inds{};
  for(auto name : names){
    temp_inds = get_qpos_indices(m, obj, name);
    qpos_inds.insert(qpos_inds.end(), temp_inds.begin(), temp_inds.end());
  }
  return qpos_inds;
}

std::vector<int> get_joint_qpos_indices(mjModel* m, const int jnt_id){
  int qpos_start{-1};
  int qpos_end{-1};

  qpos_start = m->jnt_qposadr[jnt_id];
  if (jnt_id + 1< m->njnt){
    qpos_end = m->jnt_qposadr[jnt_id + 1];
  }
  else{
    qpos_end = m->nq;
  }

  std::vector<int> qpos_inds{};
  for (int i = qpos_start; i < qpos_end; i++){
    qpos_inds.push_back(i);
  }
  return qpos_inds;
}

std::vector<int> get_joint_qpos_indices(mjModel* m, const std::string& name){
int jnt_id = mj_name2id(m, mjOBJ_JOINT, name.c_str()); // NOTE: mjOBJ_JOINT is hard-coded!
  if (jnt_id == -1)
  {
    prx_throw("Invalid joint name given: " << name)
  }

  return get_joint_qpos_indices(m, jnt_id);
}

std::vector<int> get_body_qpos_indices(mjModel* m, const std::string& name){
  int qpos_start{-1};
  int qpos_end{-1};

  int body_id = mj_name2id(m, mjOBJ_BODY, name.c_str()); // NOTE: mjOBJ_JOINT is hard-coded!
  if (body_id == -1)
  {
    prx_throw("Invalid body name given: " << name)
  }

  auto body_njnt = m->body_jntnum[body_id];
  if (body_njnt < 1)
  {
    prx_throw("Body " << name << " has no joints.");
  }
  else if(body_njnt > 1){
    prx_warn("Body " << name << " has multiple joints. Returning indices concatenated");
  }
  
  std::vector<int> body_joint_ids{};
  for(int i = 0; i < body_njnt; i++){
    body_joint_ids.push_back(m->body_jntadr[body_id + i]);
  }
  
  std::vector<int> qpos_inds{};
  for(auto body_jnt_id : body_joint_ids){
    auto new_inds = get_joint_qpos_indices(m, body_jnt_id);
    qpos_inds.insert(qpos_inds.end(), new_inds.begin(), new_inds.end());
  }
  
  return qpos_inds;
}

std::vector<int> get_body_indices(mjModel* m, const std::string& body_name)
{
  int body_id = mj_name2id(m, mjOBJ_BODY, body_name.c_str());

  if (body_id == -1)
  {
    prx_throw("Invalid body_name given.")
  }

  std::vector<int> xpos_inds{3*body_id, 3*body_id+1, 3*body_id+2};
  std::vector<int> xquat_inds{4*body_id, 4*body_id+1, 4*body_id+2, 4*body_id+3};

  xpos_inds.insert( xpos_inds.end(), xquat_inds.begin(), xquat_inds.end() );
  
  return xpos_inds;
}

std::vector<double> forward_kinematics(mjModel* m, mjData* d, const std::vector<int>& qpos_inds, 
const std::string& query_link_name, const std::vector<double>& q)
{
  prx_assert(qpos_inds.size() == q.size(), "Incorrect configuration length provided: " + 
  std::to_string(qpos_inds.size()) + ", " + std::to_string(q.size()) + "\n");
  
  // Save current joint qpos values
  double curr_qpos_vals[qpos_inds.size()] = {};
  for(int i = 0; i < qpos_inds.size(); i++){
    curr_qpos_vals[i] = d->qpos[qpos_inds[i]];
    // std::cout << curr_qpos_vals[i] << std::endl;
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
  // prx_assert(qpos_inds.size() == q->get_dim(), "Incorrect configuration length provided.")=
  
  std::vector<double> q_vec(qpos_inds.size());

  for (int i  = 0; i < q_vec.size(); i++){
    q_vec[i] = q->at(qpos_inds[i]);
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
