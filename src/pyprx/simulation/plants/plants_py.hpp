#include "pyprx/simulation/plants/two_link_acrobot_py.hpp"
#include "pyprx/simulation/plants/two_dimensional_point_py.hpp"
#include "pyprx/simulation/plants/types/types_py.hpp"
#include "pyprx/simulation/plants/pendulum_py.hpp"
#include "pyprx/simulation/plants/ackermann_FO_py.hpp"
#include "pyprx/simulation/plants/lander_LD_py.hpp"
#include "pyprx/simulation/plants/mountain_car_py.hpp"
#include "pyprx/simulation/plants/quadrotor_1d_py.hpp"

namespace pyprx
{
namespace simulation
{
namespace plants
{
void bindings()
{
  types::bindings();

  two_dimensional_point::bindings();
  acrobot::bindings();
  pendulum::bindings();
  ackermann_FO::bindings();
  lander_LD::bindings();
  mountain_car::bindings();
  quadrotor_1d::bindings();
}

}  // namespace plants
}  // namespace simulation
}  // namespace pyprx