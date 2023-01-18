#include <iostream>
#include <boost/python.hpp>
#include "pyprx/simulation/collision_checking/collision_group_py.hpp"

namespace pyprx
{
namespace simulation
{
namespace collision_checking
{
void bindings()
{
  collision_group::bindings();
}

}  // namespace collision_checking
}  // namespace simulation
}  // namespace pyprx
