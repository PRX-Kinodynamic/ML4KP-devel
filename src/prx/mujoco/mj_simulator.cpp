#ifndef MUJOCO_NOT_BUILT
#include "prx/mujoco/mj_simulator.hpp"

namespace prx
{
mujoco_simulator_t::mujoco_simulator_t(const std::string& model_path, const bool vis) : simulator_t(plant_type::MUJOCO)    
    , button_left(false)
    , button_right(false)
    , button_middle(false)
    , lastx(0)
    , lasty(0)
    , _recorded_secs(0.0)
    , _record_video(false)
    , _video_name(prx::out_path + "mj_recording.mp4")
{
  std::string full_model_path = mj_models_path + model_path;
  m = mj_loadXML(full_model_path.c_str(), NULL, NULL, 0);
  if (!m)
    prx_throw("Error in loading model.");
  d = mj_makeData(m);

  button_left = button_right = button_middle = false;
  lastx = lasty = 0; 

  _vis = MUJOCO_VIS && vis;

  // Get the simulation step from the model
  simulation_step = m->opt.timestep;
  std::cout << "Using simulation step: " << simulation_step << std::endl;

  // K TAG change this!
  if (_vis)
  {
    if (!glfwInit())
      prx_throw("Error in initializing GLFW.");

    window = glfwCreateWindow(1200, 900, "MuJoCo", NULL, NULL);
    if (!window)
      prx_throw("Error in creating GLFW window.");
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 1000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);
    viewport = { 0, 0, 1200, 900 };

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
        window, [](GLFWwindow* window) { prx_throw("Closing the visualizer will cause the simulation to crash."); });
    
    if(_record_video){
      mjv_defaultFreeCamera(m, &cam);
      cam.type = mjtCamera::mjCAMERA_TRACKING;
      // change this
      cam.trackbodyid = mj_name2id(m, mjOBJ_BODY, "body_cam");
      cam.distance = 5.0;
      cam.elevation = -60;
      std::cout << cam.azimuth << " " << cam.elevation << " " << cam.distance << std::endl;
      std::cout << "Tracking body: " << cam.trackbodyid << std::endl;
    }
  }


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
    prx_throw("This system has actuator states, which is not supported right now.");
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
  if (_vis)
    glfwTerminate();

  mj_deleteData(d);
  mj_deleteModel(m);
}

void mujoco_simulator_t::set_record_video(const bool record_video)
{
  _record_video = _vis && record_video;
};

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

  collision_groups.reset(new mujoco_collision_checker_t(sim_ptr));
  collision_groups->add_collision_group(context_name, context_systems, {});
}

void mujoco_simulator_t::step_simulation()
{
  if (_record_video)
  {
    add_frame();
  }
  // Set the warmstart acceleration to be zero (for determinism)
  for (int i = 0; i < m->nv; i++)
  {
    d->qacc_warmstart[i] = 0;
  }
  mj_step(m, d);

  if (_vis)
  {
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);

    // Refer here: https://github.com/deepmind/mujoco/issues/132
    // and here: https://roboti.us/forum/index.php?threads/rendering-geoms.3460/#post-3963
    if (goal_pos.size() != 0)
    {
      mjvGeom* goal_geom = scn.geoms + scn.ngeom++;
      mjv_initGeom(goal_geom, mjGEOM_SPHERE, NULL, NULL, NULL, NULL);
      goal_geom->rgba[0] = 0.0;
      goal_geom->rgba[1] = 1.0;
      goal_geom->rgba[2] = 0.0;
      goal_geom->rgba[3] = 0.25;
      goal_geom->size[0] = goal_radius;
      goal_geom->size[1] = goal_radius;
      goal_geom->size[2] = goal_radius;
      goal_geom->pos[0] = goal_pos[0];
      goal_geom->pos[1] = goal_pos[1];
      goal_geom->pos[2] = goal_pos[2];

      // TODO: Add a quat2euler to visualize the orientation
    }

    mjr_render(viewport, &scn, &con);
    glfwSwapBuffers(window);
    glfwPollEvents();
  }
}

void mujoco_simulator_t::step_simulation(const int step_type)
{
  if (_record_video)
  {
    add_frame();
  }
  // Set the warmstart acceleration to be zero (for determinism)
  for (int i = 0; i < m->nv; i++)
  {
    d->qacc_warmstart[i] = 0;
  }

  if(step_type == 0){
    mj_step(m, d);
  }
  else if(step_type == 1){
    mj_step1(m, d);
  }

  if (_vis)
  {
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);

    // Refer here: https://github.com/deepmind/mujoco/issues/132
    // and here: https://roboti.us/forum/index.php?threads/rendering-geoms.3460/#post-3963
    if (goal_pos.size() != 0)
    {
      mjvGeom* goal_geom = scn.geoms + scn.ngeom++;
      mjv_initGeom(goal_geom, mjGEOM_SPHERE, NULL, NULL, NULL, NULL);
      goal_geom->rgba[0] = 0.0;
      goal_geom->rgba[1] = 1.0;
      goal_geom->rgba[2] = 0.0;
      goal_geom->rgba[3] = 0.25;
      goal_geom->size[0] = goal_radius;
      goal_geom->size[1] = goal_radius;
      goal_geom->size[2] = goal_radius;
      goal_geom->pos[0] = goal_pos[0];
      goal_geom->pos[1] = goal_pos[1];
      goal_geom->pos[2] = goal_pos[2];

      // TODO: Add a quat2euler to visualize the orientation
    }

    mjr_render(viewport, &scn, &con);
    glfwSwapBuffers(window);
    glfwPollEvents();
  }
}

void mujoco_simulator_t::set_video_name(const std::string& video_name)
{
  _video_name = video_name;
}

void mujoco_simulator_t::add_frame()
{
  // std::cout << "adding frame for mujoco sim" << std::endl;
  // std::unique_ptr<unsigned char[]> rgb(new unsigned char[3 * width * height]);
  if (!_output_video.isOpened())
  {
    // std::cout << "before mjr_maxViewport" << std::endl;
    const auto rect = mjr_maxViewport(&con);
    // std::cout << "after mjr_maxViewport" << std::endl;
    const cv::Size vid_size(rect.width, rect.height);
    const int fourcc{ cv::VideoWriter::fourcc('m', 'p', '4', 'v') };
    _output_video.open(_video_name, fourcc, _fps, vid_size, true);
    prx_assert(_output_video.isOpened(), "Failed to open video output!");
  }
  // Only add a frame at the specified fps
  if (std::fmod(_recorded_secs, 1.0 / _fps) < prx::simulation_step)
  {
    mjrRect viewport = mjr_maxViewport(&con);
    int height = viewport.height;
    int width = viewport.width;
    glfwGetFramebufferSize(window, &width, &height);
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);
    glfwSwapBuffers(window);

    cv::Mat cv_pixels(height, width, CV_8UC3);
    mjr_readPixels(cv_pixels.data, nullptr, viewport, &con);
    cvtColor(cv_pixels, cv_pixels, cv::COLOR_RGB2BGR);
    cv::flip(cv_pixels, cv_pixels, 0);
    _output_video << cv_pixels;
  }
  _recorded_secs += prx::simulation_step;
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

void mujoco_simulator_t::add_pair(const std::vector<std::pair<std::string, std::string>>& pairs){
  for (auto pair : pairs){
    add_pair(pair);
  }
}


void mujoco_simulator_t::add_pair(std::pair<std::string, std::string> pair){
  auto cg = std::dynamic_pointer_cast<mujoco_collision_group_t>(collision_groups->get_collision_group("mujoco"));
  prx_assert(cg != nullptr, "collision_group_t could not be cast to mujoco_collision_group_t");

  cg->add_pair(pair);
}

void mujoco_collision_group_t::add_pair(std::pair<std::string, std::string> pair){
  add_pair(pair.first, pair.second);
}


void mujoco_collision_group_t::add_pair(std::string body1, std::string body2)
{
  int body_id1 = mj_name2id(sim->m, mjOBJ_BODY, body1.c_str());
  int body_id2 = mj_name2id(sim->m, mjOBJ_BODY, body2.c_str());

  if (body_id1 != -1 && body_id2 != -1){
    std::pair<int, int> pair = std::make_pair(body_id1, body_id2);
    ignored_pairs.push_back(pair);
  }
  else{
    prx_warn("Invalid geom pair: " << body1 << ", " << body2);
  }
}

bool mujoco_collision_group_t::in_collision()
{
  mj_forward(sim->m, sim->d);
  int ncon = sim->d->ncon;
  if (ncon > 0)
  {
    for (int i = 0; i < ncon; i++)
    {
      collision_body1 = mj_id2name(sim->m, mjOBJ_BODY, sim->m->geom_bodyid[sim->d->contact[i].geom1]);
      collision_body2 = mj_id2name(sim->m, mjOBJ_BODY, sim->m->geom_bodyid[sim->d->contact[i].geom2]);
      // std::cout << ignored_pairs.size() << std::endl;

      if (collision_body1.find("obs") != std::string::npos ^ collision_body2.find("obs") != std::string::npos)
      {
        return true;
      }

      else if(ignored_pairs.size() > 0){
        bool is_ignored_pair = false;
        collision_body_id1 = sim->m->geom_bodyid[sim->d->contact[i].geom1];
        collision_body_id2 = sim->m->geom_bodyid[sim->d->contact[i].geom2];
        
        for (int i = 0; i < ignored_pairs.size(); i++){
          if (collision_body_id1 == ignored_pairs[i].first && collision_body_id2 == ignored_pairs[i].second || 
              collision_body_id1 == ignored_pairs[i].second && collision_body_id2 == ignored_pairs[i].first){
            is_ignored_pair = true;
            break;
          }
        }
        if (!is_ignored_pair){
          // std::cout << collision_body1 << ", " << collision_body2 << std::endl;
          return true;
        }
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
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, 0.05 * yoffset, &scn, &cam);
}
}  // namespace prx
#endif