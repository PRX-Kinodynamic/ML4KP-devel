#include <iostream>
#include <boost/python.hpp>

#include "prx/simulation/plants/quadrotor_1d.hpp"

using namespace boost::python;

namespace pyprx
{
namespace simulation
{
namespace plants
{
namespace quadrotor_1d
{

void bindings()
{
  class_<prx::quadrotor_1d_t, std::shared_ptr<prx::quadrotor_1d_t>, bases<prx::ltv_t>>("quadrotor_1d", no_init)
      .def("__init__",
           make_constructor(&create_system_ptr<prx::quadrotor_1d_t>, default_call_policies(), (arg("path"))))
      // .def("propagate", &prx::quadrotor_1d_t::propagate)
      // .def("update_configuration", &prx::quadrotor_1d_t::update_configuration)
      // .def("linearize", &prx::quadrotor_1d_t::linearize)
      // .def("get_control", &prx::quadrotor_1d_t::get_control)
      // .def("to_string", &prx::quadrotor_1d_t::to_string)
      // .def("propagate", &prx::quadrotor_1d_t::propagate)
      // .def("apply_lqr", &prx::quadrotor_1d_t::apply_lqr)
      // .def("get_control_space", &prx::quadrotor_1d_t::get_control_space, return_internal_reference<>())
      // // .def("to_ptr", &create_ptr<prx::two_link_acrobot_t, prx::system_t> )
      ;
}
}  // namespace quadrotor_1d
}  // namespace plants
}  // namespace simulation
}  // namespace pyprx