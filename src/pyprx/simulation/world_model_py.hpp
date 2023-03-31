#include <iostream>
#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>
#include "prx/simulation/world_model.hpp"
#include "prx/simulation/system_group.hpp"

using namespace boost::python;
namespace pyprx
{
namespace simulation
{
namespace world_model
{
void bindings()
{
  class_<std::pair<std::shared_ptr<prx::system_group_t>, std::shared_ptr<prx::collision_group_t>>>("context")
      .def_readwrite("system_group",
                     &std::pair<std::shared_ptr<prx::system_group_t>, std::shared_ptr<prx::collision_group_t>>::first)
      .def_readwrite("collision_group",
                     &std::pair<std::shared_ptr<prx::system_group_t>, std::shared_ptr<prx::collision_group_t>>::second)
      // Comment to force ; to the next one
      ;

  class_<prx::world_model_t>(
      "world_model", init<std::vector<prx::system_ptr_t>, std::vector<std::shared_ptr<prx::movable_object_t>>>())
      .def("create_context", &prx::world_model_t::create_context)
      .def("get_context", &prx::world_model_t::get_context)
      .def("get_all_context_names", &prx::world_model_t::get_all_context_names)
      .def("step_simulation", &prx::world_model_t::step_simulation)
      .def("reset_simulation", &prx::world_model_t::reset_simulation)
      // Comment to force ; to the next one
      ;
}
}  // namespace world_model
}  // namespace simulation
}  // namespace pyprx