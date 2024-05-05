#include <iostream>
#include <boost/python.hpp>
#include "prx/simulation/controller.hpp"

namespace pyprx
{
namespace simulation
{
namespace controller
{
struct controller_wrap : prx::controller_t, wrapper<prx::controller_t>
{
  controller_wrap(const prx::controller_t& other) : controller_t(other){};
  controller_wrap(prx::system_ptr_t _plant, std::string _name) : prx::controller_t(_plant, _name){};
  controller_wrap(prx::system_ptr_t _plant) : prx::controller_t(_plant){};

  void compute_controls()
  {
    this->get_override("compute_controls")();
  }

  void compute_controls(prx::space_point_t& u)
  {
    if (override f = this->get_override("compute_controls"))
    {
      this->compute_controls(u);
    }
    else
    {
      controller_t::compute_controls(u);
    }
    // this->get_override("compute_controls")(u);
  }

  void compute_controls_1_default()
  {
    return this->controller_t::compute_controls();
  }

  void propagate_1(const double simulation_step)
  {
    this->get_override("propagate")(simulation_step);
  }

  void set_goal(prx::space_point_t g)
  {
    this->get_override("set_goal")(g);
  }
};

void (controller_wrap::*compute_controls_0)() = &controller_wrap::compute_controls;
void (controller_wrap::*compute_controls_1)(prx::space_point_t&) = &controller_wrap::compute_controls;

void (prx::controller_t::*set_goal_1)(const prx::space_point_t) = &prx::controller_t::set_goal;

void bindings()
{
  class_<controller_wrap, boost::noncopyable>("controller", init<prx::system_ptr_t>())
      .def(init<prx::system_ptr_t, std::string>())
      .def("get_state_space", &prx::controller_t::get_state_space, return_internal_reference<>())
      .def("get_control_space", &prx::controller_t::get_control_space, return_internal_reference<>())
      .def("compute_controls", compute_controls_0)
      .def("compute_controls", compute_controls_1)
      .def("propagate", &controller_wrap::propagate_1)
      .def("set_plan", &prx::controller_t::set_plan)
      .def("get_plan", &prx::controller_t::get_plan)
      .def("init_plan", &prx::controller_t::init_plan)
      .def("set_goal", set_goal_1);
}
}  // namespace controller
}  // namespace simulation
}  // namespace pyprx