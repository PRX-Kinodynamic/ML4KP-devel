#include <iostream>
#include <boost/python.hpp>
#include "prx/simulation/system_group.hpp"

namespace pyprx
{
namespace simulation
{
namespace system_group
{
void (prx::system_group_t::*propagate_4p0)(prx::space_point_t, prx::controller_ptr_t, prx::condition_check_t&,
                                           prx::space_point_t) = &prx::system_group_t::propagate;
void (prx::system_group_t::*propagate_4p1)(prx::space_point_t, prx::controller_ptr_t, prx::condition_check_t&,
                                           prx::trajectory_t&) = &prx::system_group_t::propagate;
void (prx::system_group_t::*propagate_plan_state)(prx::space_point_t, const prx::plan_t&,
                                                  prx::space_point_t) = &prx::system_group_t::propagate;
void (prx::system_group_t::*propagate_plan_traj)(prx::space_point_t, const prx::plan_t&,
                                                 prx::trajectory_t&) = &prx::system_group_t::propagate;
// void propagate_once(propagate_step step, space_point_t control = nullptr);
void system_group_propagate_once_0(prx::system_group_t* sg)
{
  sg->propagate_once(prx::propagate_step::MIDDLE_STEP);
}
void system_group_propagate_once_1(prx::system_group_t* sg, prx::space_point_t ctrl)
{
  sg->propagate_once(prx::propagate_step::MIDDLE_STEP, ctrl);
}

void bindings()
{
  register_ptr_to_python<std::shared_ptr<prx::system_group_t>>();

  class_<prx::system_group_t>("system_group", init<std::vector<prx::system_ptr_t>&>())
      .def("__init__", make_constructor(&init_as_ptr<prx::system_group_t, std::vector<prx::system_ptr_t>>,
                                        default_call_policies(), (args("sys_group"))))
      .def("get_state_space", &prx::system_group_t::get_state_space, return_internal_reference<>())
      .def("get_control_space", &prx::system_group_t::get_control_space, return_internal_reference<>())
      .def("propagate", propagate_4p0)
      .def("propagate", propagate_4p1)
      .def("propagate", propagate_plan_state)
      .def("propagate", propagate_plan_traj)
      .def("propagate_once", &prx::system_group_t::propagate_once)
      .def("propagate_once", system_group_propagate_once_0)
      // no-lint
      ;
}
}  // namespace system_group
}  // namespace simulation
}  // namespace pyprx