#pragma once

#include "prx/utilities/defs.hpp"
#include "prx/simulation/system.hpp"
#include "prx/simulation/controller.hpp"
#include "prx/simulation/playback/plan.hpp"
#include "prx/simulation/playback/trajectory.hpp"
#include "prx/simulation/collision_checking/collision_checker.hpp"
#include "prx/simulation/general/condition_check.hpp"
namespace prx
{
class simulator_t;
class system_group_manager_t;

class system_group_t
{
public:
  // system_group_t(const std::vector<system_ptr_t>& sys_group);
  system_group_t(const std::vector<system_ptr_t>& sys_group, plant_type p_type = plant_type::ANALYTICAL);
  ~system_group_t();

  void propagate(space_point_t start_state, const std::shared_ptr<controller_t>& ctrl, double duration, plan_t& plan,
                 trajectory_t& traj);

  template <typename State>
  void propagate(State start_state, const plan_t& plan, State result)
  {
    propagate_step p_step;
    state_space->copy_from(start_state);

    for (const plan_step_t& step : plan)
    {
      int steps = (int)((step.duration / simulation_step) + .1);
      // int i = 0;
      if (steps > 0)
      {
        propagate(steps, step.control);
      }
    }
    state_space->copy_to(result);
  }

  template <typename State>
  void propagate(State start_state, const plan_t& plan, trajectory_t& traj)
  {
    propagate_step p_step;
    traj.clear();

    state_space->copy_from(start_state);
    traj.copy_onto_back(state_space);
    for (const plan_step_t& step : plan)
    {
      // std::cout << "step: " << step << std::endl;
      int steps = (int)((step.duration / simulation_step) + .1);
      // int i = 0;
      if (steps > 0)
      {
        // std::cout << step.control << std::endl;
        propagate(steps, step.control, &traj);
      }
    }
  }

  void propagate(int steps, space_point_t control = nullptr, trajectory_t* traj = nullptr);

  void propagate(space_point_t start_state, controller_ptr_t ctrl, condition_check_t& cond_check, space_point_t result);

  void propagate(space_point_t start_state, controller_ptr_t ctrl, condition_check_t& cond_check, trajectory_t& result);

  void compute_stopping_maneuver(space_point_t start_state, std::vector<double>&, std::vector<double>&);

  inline space_t* get_state_space()
  {
    return state_space;
  }

  inline space_t* get_control_space()
  {
    return control_space;
  }

  inline space_t* get_parameter_space()
  {
    return parameter_space;
  }

  void propagate_once(propagate_step step, space_point_t control = nullptr);

protected:
  std::vector<system_ptr_t> group;
  space_t* state_space;
  space_t* control_space;
  space_t* parameter_space;
  simulator_t* sim;

  friend system_group_manager_t;
};
}  // namespace prx
