#include <iostream>
#include <boost/python.hpp>
#include "pyprx/simulation/multivalued_map/time_map_py.hpp"
#include "pyprx/simulation/multivalued_map/tm_controllers_py.hpp"
#include "pyprx/simulation/multivalued_map/tm_lqr_py.hpp"

namespace pyprx
{
namespace simulation
{
namespace multivalued_map
{
void bindings()
{
  time_map::bindings();
  time_map_controllers::bindings();
  tm_lqr::bindings();
}
}  // namespace multivalued_map
}  // namespace simulation
}  // namespace pyprx