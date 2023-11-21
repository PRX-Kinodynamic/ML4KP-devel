#include <iostream>
#include <boost/python.hpp>
#include "pyprx/simulation/playback/trajectory_py.hpp"
#include "pyprx/simulation/playback/plan_py.hpp"

namespace pyprx
{
namespace simulation
{
namespace playback
{
void bindings()
{
  plan::bindings();
  trajectory::bindings();
}

}  // namespace playback
}  // namespace simulation
}  // namespace pyprx