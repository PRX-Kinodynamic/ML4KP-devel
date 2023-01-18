#include <iostream>
#include <boost/python.hpp>

#include "prx/simulation/plants/pendulum.hpp"

using namespace boost::python;

namespace pyprx
{
namespace simulation
{
namespace plants
{
namespace pendulum
{

void bindings()
{
  class_<prx::pendulum_t, std::shared_ptr<prx::pendulum_t>, bases<prx::plant_t>>("pendulum", no_init)
      .def("__init__", make_constructor(&create_system_ptr<prx::pendulum_t>, default_call_policies(), (arg("path"))))
      // .def("propagate", &prx::pendulum_t::propagate)
      // .def("update_configuration", &prx::pendulum_t::update_configuration)
      // .def("linearize", &prx::pendulum_t::linearize)
      // .def("get_control", &prx::pendulum_t::get_control)
      // .def("to_string", &prx::pendulum_t::to_string)
      // .def("propagate", &prx::pendulum_t::propagate)
      // .def("apply_lqr", &prx::pendulum_t::apply_lqr)
      // .def("get_control_space", &prx::pendulum_t::get_control_space, return_internal_reference<>())
      // // .def("to_ptr", &create_ptr<prx::two_link_acrobot_t, prx::system_t> )
      ;
}
}  // namespace pendulum
}  // namespace plants
}  // namespace simulation
}  // namespace pyprx