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
      // Comment to force ; to the next one
      ;
}
}  // namespace pendulum
}  // namespace plants
}  // namespace simulation
}  // namespace pyprx