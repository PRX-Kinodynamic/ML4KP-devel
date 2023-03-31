#include <iostream>
#include <boost/python.hpp>
#include "prx/simulation/multivalued_map/tm_lqr.hpp"

namespace pyprx
{
namespace simulation
{
namespace multivalued_map
{
namespace tm_lqr
{

void bindings()
{
  def("pendulum_lqr", &prx::simulation::pendulum_lqr);
}
}  // namespace tm_lqr
}  // namespace multivalued_map
}  // namespace simulation
}  // namespace pyprx