#include <iostream>
#include <boost/python.hpp>
#include "prx/simulation/multivalued_map/tm_controllers.hpp"

namespace pyprx
{
namespace simulation
{
namespace multivalued_map
{
namespace time_map_controllers
{
using prx::simulation::time_map_controllers_t;

void bindings()
{
  class_<time_map_controllers_t, std::shared_ptr<time_map_controllers_t>>("time_map_controllers", no_init)
      .def("get_controller", &time_map_controllers_t::get_controller)
      .staticmethod("get_controller")
      .def("available_systems", &time_map_controllers_t::available_systems)
      .staticmethod("available_systems")
      .def("print_systems", &time_map_controllers_t::print_systems)
      .staticmethod("print_systems")
      // Comment to force ; to the next one
      ;
}
}  // namespace time_map_controllers
}  // namespace multivalued_map
}  // namespace simulation
}  // namespace pyprx