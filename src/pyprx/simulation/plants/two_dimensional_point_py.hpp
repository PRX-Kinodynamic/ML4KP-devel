#include "prx/simulation/plants/two_dimensional_point.hpp"

namespace pyprx
{
namespace simulation
{
namespace plants
{
namespace two_dimensional_point
{

void bindings()
{
  class_<prx::two_dimensional_point_t, std::shared_ptr<prx::two_dimensional_point_t>, bases<prx::system_t>>(
      "two_dimensional_point", no_init)
      .def("__init__",
           make_constructor(&create_system_ptr<prx::two_dimensional_point_t>, default_call_policies(), (arg("path"))))
      .def("propagate", &prx::two_dimensional_point_t::propagate)
      .def("update_configuration", &prx::two_dimensional_point_t::update_configuration)
      .def("set_state_space_bounds", &prx::two_dimensional_point_t::set_state_space_bounds);
}

}  // namespace two_dimensional_point
}  // namespace plants
}  // namespace simulation
}  // namespace pyprx