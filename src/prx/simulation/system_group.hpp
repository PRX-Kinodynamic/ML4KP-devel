#pragma once

#include "prx/utilities/defs.hpp"

#include "prx/planning/condition_check.hpp"
#include "prx/simulation/collision_checking/collision_checker.hpp"
#include "prx/simulation/controller.hpp"
#include "prx/simulation/playback/plan.hpp"
#include "prx/simulation/playback/trajectory.hpp"
#include "prx/simulation/system.hpp"

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

  template <typename StartState, typename EndState>
  void propagate(StartState& start_state, const plan_t& plan, EndState& result)
  {
    state_space->copy_from(start_state);

    for (const plan_step_t& step : plan)
    {
      const std::size_t steps = static_cast<std::size_t>((step.duration / simulation_step) + .1);
      control_space->copy_from(step.control);
      propagate_steps(steps);
    }
    state_space->copy_to(result);
  }

  /**
   * @brief      Propagate from a start state using a plan. A plan is a sequence of (duration, controls). The resulting
   * trajectory has 1 + \frac{\sum_{plan} duration}{prx::simulation_step} states.
   *
   * @param[in]  start_state  The start state
   * @param[in]  plan         The plan
   * @param      traj         The resulting trajectory
   *
   * @tparam     StartState   Start state of the trajectory
   */
  template <typename StartState>
  void propagate(StartState start_state, const plan_t& plan, trajectory_t& traj)
  {
    traj.clear();
    state_space->copy_from(start_state);
    traj.copy_onto_back(state_space);

    for (const plan_step_t& step : plan)
    {
      const double duration{ step.duration };
      control_space->copy_from(step.control);
      prx_assert(duration >= 0.0, "Negative duration!");
      const std::size_t steps = static_cast<std::size_t>((step.duration / simulation_step) + .1);
      propagate_steps(steps, traj);
    }
  }

  template <typename StartState, typename Control, typename EndState>
  void propagate(StartState start_state, const Control& control, const double duration, EndState& end_state)
  {
    state_space->copy_from(start_state);
    control_space->copy_from(control);

    if (duration > 0)
    {
      const std::size_t steps{ static_cast<std::size_t>((duration / prx::simulation_step) + .1) };
      propagate_steps(steps);
    }
    state_space->copy_to(end_state);
  }

  void propagate(int steps, space_point_t control = nullptr, trajectory_t* traj = nullptr);

  template <typename StartState, typename EndState>
  void propagate(const StartState& start_state, controller_ptr_t ctrl, condition_check_t& cond_check, EndState& result)
  {
    propagate_step p_step;
    state_space->copy_from(start_state);

    int i = 0;
    do
    {
      ctrl->compute_controls();
      propagate_once();
    } while (!cond_check.check());

    state_space->copy_to(result);
  }

  // TODO: trajectory could also be templated, the problem is how to make the copy efficient (avoid copy to point and
  // then to traj);
  template <typename StartState>
  void propagate(StartState start_state, controller_ptr_t ctrl, condition_check_t& cond_check, trajectory_t& result)
  {
    propagate_step p_step;

    result.clear();
    state_space->copy_from(start_state);
    result.emplace_back(state_space);

    int i = 0;
    do
    {
      ctrl->compute_controls();
      propagate_once();
      result.emplace_back(state_space);
    } while (!cond_check.check());
  }

  void compute_stopping_maneuver(space_point_t start_state, std::vector<double>&, std::vector<double>&);

  template <typename StartState, typename Control, typename EndState>
  void propagate_once(const StartState start_state, const Control& control, EndState& end_state)
  {
    state_space->copy_from(start_state);
    propagate_once(control);
    state_space->copy_to(end_state);
  }

  template <typename Control>
  void propagate_once(const Control& control, const propagate_step& step = propagate_step::FIRST_STEP)
  {
    control_space->copy_from(control);
    propagate_once(step);
  }

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

  void set_simulator(simulator_t* simulator)
  {
    sim = simulator;
  }

protected:
  void propagate_steps(const std::size_t& steps)
  {
    for (int i = 0; i < steps; i++)
    {
      propagate_once();
    }
  }

  void propagate_steps(const std::size_t& steps, trajectory_t& traj)
  {
    for (int i = 0; i < steps; i++)
    {
      propagate_once();
      traj.copy_onto_back(state_space);
    }
  }

  void propagate_once(const propagate_step& step = propagate_step::FIRST_STEP);

  std::vector<system_ptr_t> group;
  space_t* state_space;
  space_t* control_space;
  space_t* parameter_space;
  simulator_t* sim;

  friend system_group_manager_t;
};
}  // namespace prx
