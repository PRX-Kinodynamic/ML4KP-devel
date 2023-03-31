#include "prx/simulation/system_factory.hpp"

namespace pyprx
{
namespace simulation
{
namespace system_factory
{

void bindings()
{
  class_<prx::system_factory_t, boost::noncopyable>("system_factory", no_init)
      // .def<void (prx::param_loader::*)(std::vector<std::string>)>("add_opts", &prx::param_loader::add_opts)
      // .def<prx::system_ptr_t (prx::system_factory_t::*)(std::string&)>("create_system",
      // &prx::system_factory_t::create_system)
      .def("create_system", &prx::system_factory_t::create_system)
      .staticmethod("create_system")
      .def("get_system_max_velocity", &prx::system_factory_t::get_system_max_velocity)
      .staticmethod("get_system_max_velocity")
      .def("available_systems", &prx::system_factory_t::available_systems)
      .staticmethod("available_systems")
      .def("available_velocity_functions", &prx::system_factory_t::get_system_max_velocity)
      .staticmethod("available_velocity_functions");
}
}  // namespace system_factory
}  // namespace simulation
}  // namespace pyprx