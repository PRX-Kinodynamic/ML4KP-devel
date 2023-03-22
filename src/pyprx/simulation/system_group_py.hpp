#include <iostream>
#include <boost/python.hpp>
#include "prx/simulation/system_group.hpp"

namespace pyprx
{
namespace simulation
{
namespace system_group
{
void (prx::system_group_t::*propagate_4p1)(prx::space_point_t, prx::controller_ptr_t, prx::condition_check_t&,
                                           prx::trajectory_t&) = &prx::system_group_t::propagate;
// void (prx::system_group_t::*propagate_plan_state)(prx::space_point_t, const prx::plan_t&,
//                                                   prx::space_point_t) = &prx::system_group_t::propagate;
void (prx::system_group_t::*propagate_plan_traj)(prx::space_point_t, const prx::plan_t&,
                                                 prx::trajectory_t&) = &prx::system_group_t::propagate;

void system_group_propagate_once_1(prx::system_group_t* sg, prx::space_point_t ctrl)
{
  sg->propagate_once(ctrl, prx::propagate_step::MIDDLE_STEP);
}
void system_group_propagate_1(prx::system_group_t* sg, const prx::space_point_t& start_state,
                              prx::controller_ptr_t ctrl_ptr, prx::condition_check_t& cond_check,
                              prx::space_point_t& end_state)
{
  sg->propagate(start_state, ctrl_ptr, cond_check, end_state);
}
void system_group_propagate_2(prx::system_group_t* sg, const prx::space_point_t& start_state, const prx::plan_t& plan,
                              prx::space_point_t& end_state)
{
  sg->propagate(start_state, plan, end_state);
}

void bindings()
{
  register_ptr_to_python<std::shared_ptr<prx::system_group_t>>();

  class_<prx::system_group_t>("system_group", init<std::vector<prx::system_ptr_t>&>())
      .def("__init__", make_constructor(&init_as_ptr<prx::system_group_t, std::vector<prx::system_ptr_t>>,
                                        default_call_policies(), (args("sys_group"))))
      .def("get_state_space", &prx::system_group_t::get_state_space, return_internal_reference<>())
      .def("get_control_space", &prx::system_group_t::get_control_space, return_internal_reference<>())
      .def("propagate", system_group_propagate_1)
      .def("propagate", system_group_propagate_2)
      // overload_member_function<prx::system_group_t, const prx::space_point_t&, prx::controller_ptr_t,
      //                          prx::condition_check_t&, prx::space_point_t&, &prx::system_group_t::propagate>)
      .def("propagate", propagate_4p1)
      .def("propagate", propagate_plan_traj)
      .def("propagate_once",
           &prx::system_group_t::propagate_once<prx::space_point_t, prx::space_point_t, prx::space_point_t>)
      .def("propagate_once", &prx::system_group_t::propagate_once<prx::space_point_t>)
      // no-lint
      ;
}
}  // namespace system_group
}  // namespace simulation
}  // namespace pyprx