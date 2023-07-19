#include "pyprx/simulation/controllers/lqr_py.hpp"
#include "pyprx/simulation/controllers/ackermann_FO_ctrl_py.hpp"
#include "pyprx/simulation/controllers/noisy_controller_py.hpp"
#include "pyprx/simulation/controllers/bang_bang_py.hpp"

void pyprx_simulation_controllers()
{
  pyprx_simulation_controllers_lqr();
  pyprx_simulation_controllers_ackermann_FO_ctrl();
  pyprx_simulation_controllers_noisy();
  pyprx_simulation_controllers_bang_bang();
}