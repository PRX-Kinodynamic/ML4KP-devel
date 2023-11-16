#pragma once

#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/condition_check.hpp"

#include "prx/simulation/system.hpp"
#include "prx/simulation/controller.hpp"
#include "prx/simulation/playback/plan.hpp"
#include "prx/simulation/playback/trajectory.hpp"
#include "prx/simulation/collision_checking/collision_checker.hpp"
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

  void propagate(space_point_t start_state, const plan_t& plan, space_point_t result);

  void propagate(space_point_t start_state, const plan_t& plan, trajectory_t& traj);

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
    return _parameter_space;
  }

  void propagate_once(space_point_t control = nullptr);

  std::vector<system_ptr_t>::iterator begin()
  {
    return group.begin();
  }

  std::vector<system_ptr_t>::iterator end()
  {
    return group.end();
  }

protected:
  std::vector<system_ptr_t> group;
  space_t* state_space;
  space_t* control_space;
  space_t* _parameter_space;
  simulator_t* sim;

  friend system_group_manager_t;
};
}  // namespace prx
