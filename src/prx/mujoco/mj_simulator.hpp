#pragma once
#ifndef MUJOCO_NOT_BUILT
#include "prx/simulation/simulator.hpp"
#include "prx/simulation/playback/plan.hpp"
#include "prx/simulation/collision_checking/collision_checker.hpp"

#include "prx/mujoco/mj_utils.hpp"
// #include "prx/mujoco/mj_plant.hpp"

#include "GLFW/glfw3.h"
#include "mujoco/mujoco.h"

namespace prx
{
// class mujoco_plant_t;
class mujoco_simulator_t : public simulator_t
{
private:
  mjvCamera cam;
  mjvOption opt;
  mjvScene scn;
  mjrContext con;

  GLFWwindow* window;
  bool _visualize;

protected:
  bool button_left, button_right, button_middle;
  double lastx, lasty;

  void mouse_button(GLFWwindow* window, int button, int act, int mods);
  void mouse_move(GLFWwindow* window, double xpos, double ypos);
  void scroll(GLFWwindow* window, double xoffset, double yoffset);

public:
  mujoco_simulator_t(const std::string& model_path, bool visualize = true);

  virtual ~mujoco_simulator_t();

  void toggle_visualizer()
  {
    _visualize = !_visualize;
  }

  void set_visaliztion(const bool visualize)
  {
    _visualize = visualize;
  }
  void init_simulator();

  virtual void step_simulation(propagate_step step) override;

  virtual void reset_simulation() override;

  bool in_collision();

  void set_state(const MujocoState& state);

  MujocoState get_state();

  mjModel* m;
  mjData* d;

  std::vector<mjJointInfo*> joint_info;
  std::vector<mjActuatorInfo*> actuator_info;

  std::vector<double*> actuator_internal_state;
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

protected:
  std::shared_ptr<mujoco_simulator_t> sim;
  std::string collision_body1, collision_body2;
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