#include "prx/simulation/loaders/obstacle_loader.hpp"

namespace pyprx
{
namespace simulation
{
namespace loaders
{
namespace obstacle_loader
{

void bindings()
{
  class_<prx::obstacle_loader_t, std::shared_ptr<prx::obstacle_loader_t>>("obstacle_loader", no_init)
      .def("__init__",
           make_constructor(&init_as_ptr<prx::obstacle_loader_t, const std::string&>, default_call_policies()))
      .def("get_names", &prx::obstacle_loader_t::get_names)
      .def("get_obstacles", &prx::obstacle_loader_t::get_obstacles);
}

}  // namespace obstacle_loader
}  // namespace loaders
}  // namespace simulation
}  // namespace pyprx