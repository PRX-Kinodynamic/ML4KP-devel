#include "prx/simulation/time_map/tm_controllers.hpp"

namespace prx
{
namespace simulation
{

time_map_controllers_t& time_map_controllers_t::get()
{
  static time_map_controllers_t instance;
  return instance;
}

void time_map_controllers_t::print_systems()
{
  std::cout << "[time_map_controllers_t systems]: \n";
  for (auto it : time_map_controllers_t::get().controller_generators)
  {
    std::cout << it.first << "\n";
  }
}

bool time_map_controllers_t::register_system(const std::string name, tm_controller_gen_fn func)
{
  bool res;
  auto it = time_map_controllers_t::get().controller_generators.find(name);
  if (it != time_map_controllers_t::get().controller_generators.end())
  {
    res = true;
  }
  else
  {
    res = controller_generators.insert(std::make_pair(name, func)).second;
    prx_assert(res, "Problem registering system: " << name);
  }
  return res;
}

}  // namespace simulation
}  // namespace prx