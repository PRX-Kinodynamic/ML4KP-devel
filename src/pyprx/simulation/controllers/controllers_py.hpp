#include "pyprx/simulation/controllers/lqr_py.hpp"
#include "pyprx/simulation/controllers/ackermann_FO_ctrl_py.hpp"
#include "pyprx/simulation/controllers/noisy_controller_py.hpp"
#include "pyprx/simulation/controllers/bang_bang_py.hpp"

namespace pyprx
{
namespace simulation
{
namespace controllers
{
void bindings()
{
  lqr::bindings();
  ackermann_FO_ctrl::bindings();
  noisy_controller::bindings();
  bang_bang::bindings();
}
}  // namespace controllers
}  // namespace simulation
}  // namespace pyprx