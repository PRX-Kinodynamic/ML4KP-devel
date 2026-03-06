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
  using SystemGroupPtr = std::shared_ptr<system_group_t>;

  // system_group_t(const std::vector<system_ptr_t>& sys_group);
  system_group_t(const std::vector<system_ptr_t>& sys_group, plant_type p_type = plant_type::ANALYTICAL);
  ~system_group_t();

  void propagate(space_point_t start_state, const plan_t& plan, space_point_t result);

  void propagate(space_point_t start_state, const plan_t& plan, trajectory_t& traj);

  void propagate(int steps, space_point_t control = nullptr, trajectory_t* traj = nullptr);

  template <typename Start, typename Ctrl, typename Check, typename Result,
            std::enable_if_t<not prx::utilities::is_any_ptr<Start>::value, bool> = true,
            std::enable_if_t<not prx::utilities::is_any_ptr<Ctrl>::value, bool> = true,
            std::enable_if_t<not prx::utilities::is_any_ptr<Check>::value, bool> = true,
            std::enable_if_t<not prx::utilities::is_any_ptr<Result>::value, bool> = true>
  void propagate(const Start& start_state, Ctrl& ctrl, Check& check, Result& result)
  {
    state_space->copy_from(start_state);

    do
    {
      ctrl();
      propagate_once(nullptr);
    } while (not check());

    state_space->copy_to(result);
  }

  void propagate(space_point_t start_state, controller_ptr_t ctrl, condition_check_t& cond_check, space_point_t result);

  void propagate(space_point_t start_state, controller_ptr_t ctrl, condition_check_t& cond_check, trajectory_t& result);

  void steer(space_point_t, const space_point_t, const space_point_t, const double,
             distance_function_t distance_function);

  void steer(trajectory_t&, const space_point_t, const space_point_t, const double,
             distance_function_t distance_function);

  void steer_once(const space_point_t, const space_point_t, const double);

  void sense()
  {
    for (auto& s : group)
    {
      s->sense();
    }
  }

  // void system_group_t::compute_stopping_maneuver(space_point_t start_state, double& time)
  void compute_stopping_maneuver(space_point_t start_state, double& time)
  {
    prx_assert(group.size() == 1,
               "[system_group_t::compute_stopping_maneuver] Expected group of size 1 but got " << group.size());
    group[0]->compute_stopping_maneuver(start_state, time);
  }
  // void compute_stopping_maneuver(space_point_t start_state, std::vector<double>&, std::vector<double>&);

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

  inline space_t* get_sensor_space()
  {
    return _sensor_space;
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
  space_t* _sensor_space;
  simulator_t* sim;

  friend system_group_manager_t;
};
}  // namespace prx
