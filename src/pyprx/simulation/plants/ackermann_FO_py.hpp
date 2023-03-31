#include <iostream>
#include <boost/python.hpp>

#include "prx/simulation/plants/ackermann_FO.hpp"

using namespace boost::python;

namespace pyprx
{
namespace simulation
{
namespace plants
{
namespace ackermann_FO
{

void bindings()
{
  class_<prx::ackermann_FO, std::shared_ptr<prx::ackermann_FO>, bases<prx::plant_t>>("ackermann_FO", no_init)
      .def("__init__", make_constructor(&create_system_ptr<prx::ackermann_FO>, default_call_policies(), (arg("path"))));
}

}  // namespace ackermann_FO
}  // namespace plants
}  // namespace simulation
}  // namespace pyprx