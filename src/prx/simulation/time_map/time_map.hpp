#pragma once
#include "prx/simulation/system.hpp"
#include "prx/simulation/time_map/tm_controllers.hpp"
namespace prx
{
namespace simulation
{
class time_map_t
{
public:
  time_map_t(const std::string system_name, const system_ptr_t system_ptr, const std::shared_ptr<system_group_t> sg)
    : _sg(sg), _system_name(system_name), _system(system_ptr), _checker("sim_time", 1)
  {
    _ss = _sg->get_state_space();
    _cs = _sg->get_control_space();
    x_goal = _ss->make_point();
    u_goal = _cs->make_point();
    // _ps = _sg->get_parameter_space();
    auto controller_generator = time_map_controllers_t::get_controller(system_name);
    _controller = controller_generator(*this);
  }

  void set_duration(const double duration)
  {
    _checker.set_check_value(duration);
  }

  template <typename State>
  void operator()(const State start, State& result)
  {
    _checker.reset();
    _sg->propagate(start, _controller, _checker, result);
  }

  template <typename State, typename Trajectory>
  void operator()(const State start, Trajectory& trajectory)
  {
    _checker.reset();
    _sg->propagate(start, _controller, _checker, trajectory);
  }

  system_ptr_t _system;
  space_point_t x_goal;
  space_point_t u_goal;

private:
  std::string _system_name;
  space_t* _ss;
  space_t* _cs;
  space_t* _ps;
  std::shared_ptr<system_group_t> _sg;

  condition_check_t _checker;
  controller_ptr_t _controller;
};

}  // namespace simulation
}  // namespace prx
