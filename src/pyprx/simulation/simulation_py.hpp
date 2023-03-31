#include <iostream>
#include <boost/python.hpp>

#include "prx/simulation/defs.hpp"

#include "pyprx/simulation/collision_checking/collision_checking_py.hpp"
#include "pyprx/simulation/controller_py.hpp"
#include "pyprx/simulation/controllers/controllers_py.hpp"
#include "pyprx/simulation/multivalued_map/multivalued_map_py.hpp"
#include "pyprx/simulation/loaders/loaders_py.hpp"
#include "pyprx/simulation/plant_py.hpp"
#include "pyprx/simulation/system_factory_py.hpp"
#include "pyprx/simulation/system_group_py.hpp"
#include "pyprx/simulation/system_py.hpp"
#include "pyprx/simulation/plants/plants_py.hpp"
#include "pyprx/simulation/playback/playback_py.hpp"
#include "pyprx/simulation/world_model_py.hpp"

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

  collision_checking::bindings();
  controller::bindings();
  controllers::bindings();

  multivalued_map::bindings();

  loaders::bindings();

  system::bindings();
  system_factory::bindings();
  system_group::bindings();

  plant::bindings();
  plants::bindings();

  playback::bindings();
  world_model::bindings();
}

}  // namespace simulation
}  // namespace pyprx