#ifndef MUJOCO_NOT_BUILT
#include "prx/mujoco/mj_plant.hpp"
#include "prx/mujoco/mj_simulator.hpp"

namespace prx
{
mujoco_simulator_t::mujoco_simulator_t(const std::string& model_path, bool visualize)
  : simulator_t(plant_type::MUJOCO), _visualize(visualize)
{
  std::string full_model_path = mj_models_path + model_path;
  m = mj_loadXML(full_model_path.c_str(), NULL, NULL, 0);
  if (!m)
    prx_throw("Error in loading model.") d = mj_makeData(m);

  button_left = button_right = button_middle = false;
  lastx = lasty = 0;

  // Get the simulation step from the model
  simulation_step = m->opt.timestep;
  std::cout << "Using simulation step: " << simulation_step << std::endl;

  // if (_visualize)
  // {
  if (!glfwInit())
    prx_throw("Error in initializing GLFW.");

  window = glfwCreateWindow(1200, 900, "MuJoCo", NULL, NULL);
  if (!window)
    prx_throw("Error in creating GLFW window.") glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);
  mjv_makeScene(m, &scn, 1000);
  mjr_makeContext(m, &con, mjFONTSCALE_150);

  glfwSetWindowUserPointer(window, this);
  glfwSetCursorPosCallback(window, [](GLFWwindow* window, double xpos, double ypos) {
    auto sim = static_cast<mujoco_simulator_t*>(glfwGetWindowUserPointer(window));
    sim->mouse_move(window, xpos, ypos);
  });
  glfwSetMouseButtonCallback(window, [](GLFWwindow* window, int button, int action, int mods) {
    auto sim = static_cast<mujoco_simulator_t*>(glfwGetWindowUserPointer(window));
    sim->mouse_button(window, button, action, mods);
  });
  glfwSetScrollCallback(window, [](GLFWwindow* window, double xoffset, double yoffset) {
    auto sim = static_cast<mujoco_simulator_t*>(glfwGetWindowUserPointer(window));
    sim->scroll(window, xoffset, yoffset);
  });
  glfwSetWindowCloseCallback(
      window, [](GLFWwindow* window) { prx_throw("Closing the visualizer will cause the simulation to crash.") });
  // }
  // number of generalized coordinates
  std::cout << "nq = " << m->nq << std::endl;
  // number of degrees of freedom
  std::cout << "nv = " << m->nv << std::endl;
  // number of actuators/controls
  std::cout << "nu = " << m->nu << std::endl;
  // number of activation states
  std::cout << "na = " << m->na << std::endl;

  if (m->na != 0)
  {
    prx_throw("This system has actuator states, which is not supported right now.")
  }

  get_mj_joint_info(m, joint_info);
  for (auto& info : joint_info)
  {
    std::cout << *info << std::endl;
  }

  get_mj_actuator_info(m, actuator_info);
  for (auto& info : actuator_info)
  {
    std::cout << *info << std::endl;
  }
}

mujoco_simulator_t::~mujoco_simulator_t()
{
  mjv_freeScene(&scn);
  mjr_freeContext(&con);
  glfwTerminate();

  mj_deleteData(d);
  mj_deleteModel(m);
  mj_deactivate();
}

void mujoco_simulator_t::init_simulator()
{
  system_groups->link_simulator(this);
  std::string context_name = "mujoco";
  std::vector<system_ptr_t> context_systems;

  system_ptr_t system;
  system.reset(new mujoco_plant_t("mujoco_plant"));
  context_systems.push_back(system);
  auto mj_ptr = std::dynamic_pointer_cast<mujoco_plant_t>(system);
  auto sim_ptr = std::static_pointer_cast<mujoco_simulator_t>(this->shared_ptr());
  mj_ptr->initialize(sim_ptr);

  system_groups->add_system_group(context_name, context_systems);

  // TODO: Need to set up collision stuff here.
  collision_groups.reset(new mujoco_collision_checker_t(sim_ptr));
  collision_groups->add_collision_group(context_name, context_systems, {});
}

void mujoco_simulator_t::step_simulation(propagate_step step)
{
  // Set the warmstart acceleration to be zero (for determinism)
  for (int i = 0; i < m->nv; i++)
  {
    d->qacc_warmstart[i] = 0;
  }
  mj_step(m, d);
  if (_visualize)
  {
    mjrRect viewport = { 0, 0, 0, 0 };
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);
    glfwSwapBuffers(window);
    glfwPollEvents();
  }
}

void mujoco_simulator_t::reset_simulation()
{
}

MujocoState mujoco_simulator_t::get_state()
{
  MujocoState s;
  s.time = d->time;
  for (int i = 0; i < m->nq; i++)
  {
    s.qpos.push_back(d->qpos[i]);
  }
  for (int i = 0; i < m->nv; i++)
  {
    s.qvel.push_back(d->qvel[i]);
  }
  for (int i = 0; i < m->na; i++)
  {
    s.act.push_back(d->act[i]);
  }
  for (int i = 0; i < m->nu; i++)
  {
    s.ctrl.push_back(d->ctrl[i]);
  }
  return s;
}

bool mujoco_collision_group_t::in_collision()
{
  int ncon = sim->d->ncon;
  if (ncon > 0)
  {
    for (int i = 0; i < ncon; i++)
    {
      collision_body1 = mj_id2name(sim->m, mjOBJ_BODY, sim->m->geom_bodyid[sim->d->contact[i].geom1]);
      collision_body2 = mj_id2name(sim->m, mjOBJ_BODY, sim->m->geom_bodyid[sim->d->contact[i].geom2]);
      if (collision_body1.find("obs") != std::string::npos ^ collision_body2.find("obs") != std::string::npos)
      {
        return true;
      }
    }
  }
  return false;
}

void mujoco_simulator_t::mouse_button(GLFWwindow* window, int button, int act, int mods)
{
  button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
  button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
  button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

  glfwGetCursorPos(window, &lastx, &lasty);
}

void mujoco_simulator_t::mouse_move(GLFWwindow* window, double xpos, double ypos)
{
  // no buttons down: nothing to do
  if (!button_left && !button_middle && !button_right)
  {
    return;
  }

  // compute mouse displacement, save
  double dx = xpos - lastx;
  double dy = ypos - lasty;
  lastx = xpos;
  lasty = ypos;

  // get current window size
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  // get shift key state
  bool mod_shift =
      (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

  // determine action based on mouse button
  mjtMouse action;
  if (button_right)
  {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  }
  else if (button_left)
  {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  }
  else
  {
    action = mjMOUSE_ZOOM;
  }

  // move camera
  mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

void mujoco_simulator_t::scroll(GLFWwindow* window, double xoffset, double yoffset)
{
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}
}  // namespace prx
#endif
