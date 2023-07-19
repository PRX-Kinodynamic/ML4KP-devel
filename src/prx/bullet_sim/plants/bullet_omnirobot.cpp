#include "prx/bullet_sim/plants/bullet_omnirobot.hpp"

namespace prx
{
bullet_omnirobot_t::bullet_omnirobot_t(const std::string& path) : bullet_plant_t(path)
{
  x = y = z = r = p = yaw = dx = dy = dz = dr = dp = dyaw = sid = 0;
  z = 1;
  state_memory = { &x, &y, &z, &r, &p, &yaw, &dx, &dy, &dz, &dr, &dp, &dyaw, &sid };
  state_space = new space_t("EEERRREEEEEED", state_memory, "XYZRPYdxdydzdrdpdyaId");
  state_bounds_l = { -15, -15, -.1, -3.14, -3.14, -3.14, -20, -20, -20, -20, -20, -20, 0 };
  state_bounds_u = { 15, 15, 1, 3.14, 3.14, 3.14, 20, 20, 20, 20, 20, 20, PRX_INFINITY };
  state_space->set_bounds(state_bounds_l, state_bounds_u);

  w1 = w2 = w3 = w4 = 0;
  control_memory = { &w1, &w2, &w3, &w4 };
  input_control_space = new space_t("EEEE", control_memory, "ctrl_space");
  control_bounds_l = { -2, -2, -2, -2 };
  control_bounds_u = { 2, 2, 2, 2 };
  input_control_space->set_bounds(control_bounds_l, control_bounds_u);

  // @aravind: Will this work if it is moved to the bullet plant constructor?
  current_control = input_control_space->make_point();
  current_state = state_space->make_point();
}

void bullet_omnirobot_t::reset()
{
  btVector3 basePosition, baseRotation;
  btQuaternion baseOrientation;

  basePosition[0] = state_space->at(0);
  basePosition[1] = state_space->at(1);
  basePosition[2] = state_space->at(2);
  baseRotation[0] = state_space->at(3);
  baseRotation[1] = state_space->at(4);
  baseRotation[2] = state_space->at(5);
  bullet_simulator_t::get_quaternion_from_euler(baseOrientation, baseRotation);

  loadURDFArgs.m_startPosition = basePosition;
  loadURDFArgs.m_startOrientation = baseOrientation;
  uniqueId = sim->loadURDF(robot_model_path, loadURDFArgs);

  int numJoints = sim->getNumJoints(uniqueId);
  b3JointInfo jointInfo;
  for (int i = 0; i < numJoints; i++)
  {
    sim->getJointInfo(uniqueId, i, &jointInfo);
    std::string joint_name(jointInfo.m_jointName);
    if (joint_name == "base_ow_1_axis")
    {
      wheelJoints.push_back(i);
    }
    else if (joint_name == "base_ow_2_axis")
    {
      wheelJoints.push_back(i);
    }
    else if (joint_name == "base_ow_3_axis")
    {
      wheelJoints.push_back(i);
    }
    else if (joint_name == "base_ow_4_axis")
    {
      wheelJoints.push_back(i);
    }
    b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_VELOCITY);

    controlArgs.m_targetVelocity = 0;
    controlArgs.m_maxTorqueValue = 0;
    sim->setJointMotorControl(uniqueId, i, controlArgs);
  }

  for (int i = 0; i < 100; i++)
  {
    sim->stepSimulation();
  }
}

void bullet_omnirobot_t::initialize(std::shared_ptr<b3RobotSimulatorClientAPI> _sim)
{
  sim = _sim;
  reset();
}

bullet_omnirobot_t::~bullet_omnirobot_t()
{
}

void bullet_omnirobot_t::compute_control()
{
  input_control_space->copy_to_point(current_control);

  b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_VELOCITY);
  controlArgs.m_maxTorqueValue = maxForce;

  for (int i = 0; i < wheelJoints.size(); i++)
  {
    controlArgs.m_targetVelocity = current_control->at(i);
    sim->setJointMotorControl(uniqueId, wheelJoints[i], controlArgs);
  }
}

void bullet_omnirobot_t::update_from_bullet(const bool save_sim_state)
{
  current_state_vec.clear();
  btVector3 basePosition, baseRotation;
  btQuaternion baseOrientation;

  sim->getBasePositionAndOrientation(uniqueId, basePosition, baseOrientation);
  bullet_simulator_t::get_euler_from_quaternion(baseRotation, baseOrientation);

  current_state_vec.push_back(basePosition[0]);
  current_state_vec.push_back(basePosition[1]);
  current_state_vec.push_back(basePosition[2]);

  current_state_vec.push_back(baseRotation[0]);
  current_state_vec.push_back(baseRotation[1]);
  current_state_vec.push_back(baseRotation[2]);

  btVector3 baseVel, baseAngVel;
  sim->getBaseVelocity(uniqueId, baseVel, baseAngVel);
  current_state_vec.insert(current_state_vec.end(),
                           { baseVel[0], baseVel[1], baseVel[2], baseAngVel[0], baseAngVel[1], baseAngVel[2] });

  int sid = get_state_id();

  if (save_sim_state)
    sid = sim->saveStateToMemory();
  current_state_vec.push_back(sid);
  lastSavedId = std::max(lastSavedId, sid);
  state_space->copy_from_vector(current_state_vec);
  state_space->copy_point_from_vector(current_state, current_state_vec);
}
}  // namespace prx
