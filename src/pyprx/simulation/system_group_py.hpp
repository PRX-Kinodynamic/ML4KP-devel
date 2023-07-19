#include <iostream>
#include <boost/python.hpp>
#include "prx/simulation/system_group.hpp"

void (prx::system_group_t::*propagate_4p0)(prx::space_point_t, prx::controller_ptr_t, prx::condition_check_t&,
                                           prx::space_point_t) = &prx::system_group_t::propagate;
void (prx::system_group_t::*propagate_4p1)(prx::space_point_t, prx::controller_ptr_t, prx::condition_check_t&,
                                           prx::trajectory_t&) = &prx::system_group_t::propagate;

void pyprx_simulation_system_group_py()
{
  register_ptr_to_python<std::shared_ptr<prx::system_group_t>>();

  class_<prx::system_group_t>("system_group", init<std::vector<prx::system_ptr_t>&>())
      .def("__init__", make_constructor(&init_as_ptr<prx::system_group_t, std::vector<prx::system_ptr_t>>,
                                        default_call_policies(), (args("sys_group"))))
      // .def("create_ptr", &create_ptr<prx::system_group_t, prx::system_group_t> )
      // .def("__init__", make_constructor(&create_ptr<prx::system_group_t, prx::system_group_t>,
      // default_call_policies() ))
      .def("get_state_space", &prx::system_group_t::get_state_space, return_internal_reference<>())
      .def("get_control_space", &prx::system_group_t::get_control_space, return_internal_reference<>())
      .def("propagate", propagate_4p0)
      .def("propagate", propagate_4p1);
}