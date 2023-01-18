#include <iostream>
#include <boost/python.hpp>
#include "prx/simulation/defs.hpp"
#include "pyprx/simulation/plant_py.hpp"
#include "pyprx/simulation/system_py.hpp"
#include "pyprx/simulation/controller_py.hpp"
#include "pyprx/simulation/system_group_py.hpp"
#include "pyprx/simulation/plants/plants_py.hpp"
#include "pyprx/simulation/system_factory_py.hpp"
#include "pyprx/simulation/loaders/loaders_py.hpp"
#include "pyprx/simulation/playback/playback_py.hpp"
#include "pyprx/simulation/controllers/controllers_py.hpp"
#include "pyprx/simulation/collision_checking/collision_checking_py.hpp"

namespace pyprx
{
namespace simulation
{

double get_simulation_step()
{
  return prx::simulation_step;
}

void set_simulation_step(const double& ss)
{
  prx::simulation_step = ss;
}

void bindings()
{
  def("set_simulation_step", &set_simulation_step);
  def("get_simulation_step", &get_simulation_step);

  system::bindings();
  pyprx_simulation_plant();
  system_group::bindings();
  pyprx_simulation_collision_checking_py();

  pyprx_simulation_controller();
  pyprx_simulation_controllers();

  pyprx_simulation_plants();
  playback::bindings();

  pyprx_simulation_loaders();
  pyprx_simulation_system_factory_py();
}

}  // namespace simulation
}  // namespace pyprx