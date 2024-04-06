#pragma once
#ifndef MUJOCO_NOT_BUILT
#include "prx/simulation/simulator.hpp"
#include "prx/simulation/playback/plan.hpp"
#include "prx/simulation/collision_checking/collision_checker.hpp"
#include "prx/mujoco/mj_utils.hpp"
#include "prx/mujoco/mj_plant.hpp"

#include "GLFW/glfw3.h"
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include "mujoco/mujoco.h"

namespace prx
{
class mujoco_plant_t;
class mujoco_simulator_t : public simulator_t
{
private:
  bool _record_video;
  double _recorded_secs;
  const double _fps{ 30 };
  std::string _video_name;
  mjvCamera cam;
  mjvOption opt;
  mjvScene scn;
  mjrContext con;

  GLFWwindow* window;

protected:
  bool button_left, button_right, button_middle;
  double lastx, lasty;
  mjrRect viewport;
  std::vector<double> goal_pos;
  double goal_radius;

  void mouse_button(GLFWwindow* window, int button, int act, int mods);
  void mouse_move(GLFWwindow* window, double xpos, double ypos);
  void scroll(GLFWwindow* window, double xoffset, double yoffset);

public:
  mujoco_simulator_t()
    : simulator_t(plant_type::MUJOCO)
    , button_left(false)
    , button_right(false)
    , button_middle(false)
    , lastx(0)
    , lasty(0)
    , _recorded_secs(0.0)
    , _record_video(false)
    , _video_name(prx::out_path + "mj_recording.mp4")
  {
  }

  mujoco_simulator_t(const std::string& model_path);

  virtual ~mujoco_simulator_t();

  void init_simulator(const int n_ignored_pairs=0, std::vector<std::pair<std::string, std::string>>* ignored_pairs=nullptr);

  virtual void step_simulation() override;

  virtual void reset_simulation() override;

  void add_frame();

  inline void set_goal(const std::vector<double> goal)
  {
    goal_pos.clear();
    for (int i = 0; i < 3; i++)
    {
      goal_pos.push_back(goal.at(i));
    }
  }

  inline void set_cam_elevation(const double elevation)
  {
    cam.elevation = elevation;
  }

  inline void set_cam_distance(const double distance)
  {
    cam.distance = distance;
  }

  inline void set_cam_azimuth(const double azimuth)
  {
    cam.azimuth = azimuth;
  }

  inline void set_goal_radius(double radius)
  {
    goal_radius = radius;
  }

  bool in_collision();

  void set_state(const MujocoState& state);

  void set_record_video(const bool record_video);

  void set_video_name(const std::string& video_name);

  MujocoState get_state();

  void close_video()
  {
    if (_output_video.isOpened())
      _output_video.release();
  }

  mjModel* m;
  mjData* d;

  std::vector<mjJointInfo*> joint_info;
  std::vector<mjActuatorInfo*> actuator_info;

  std::vector<double*> actuator_internal_state;
  cv::VideoWriter _output_video;
};

class mujoco_collision_group_t : public collision_group_t
{
public:
  mujoco_collision_group_t(const std::vector<system_ptr_t>& in_plants,
                           const std::vector<std::shared_ptr<movable_object_t>>& in_obstacles = {})
  {
    prx_warn(
        "Default constructor for mujoco collision group is not implemented. Please use the constructor with the mujoco "
        "simulator.");
  }
  mujoco_collision_group_t(const std::shared_ptr<mujoco_simulator_t>& sim)
  {
    this->sim = sim;
  }
  virtual ~mujoco_collision_group_t(){};

  bool in_collision() override;

  int n_ignored_pairs;
  std::vector<std::pair<std::string, std::string>>* ignored_pairs;

protected:
  std::shared_ptr<mujoco_simulator_t> sim;
  std::string collision_body1, collision_body2;
  std::pair<std::string, std::string> collision_pair;
};

class mujoco_collision_checker_t : public collision_checker_t
{
public:
  mujoco_collision_checker_t(const std::shared_ptr<mujoco_simulator_t>& _sim) : collision_checker_t()
  {
    sim = _sim;
  }
  virtual ~mujoco_collision_checker_t(){};

  virtual void add_collision_group(const std::string& group_name, const std::vector<system_ptr_t>& in_plants,
                                   const std::vector<std::shared_ptr<movable_object_t>>& in_obstacles) override
  {
    collision_groups[group_name] = std::make_shared<mujoco_collision_group_t>(sim);
  }

protected:
  std::shared_ptr<mujoco_simulator_t> sim;
};

}  // namespace prx
#endif