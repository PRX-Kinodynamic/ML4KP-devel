#include "prx/simulation/collision_checking/collision_group.hpp"

namespace pyprx
{
namespace simulation
{
namespace collision_checking
{
namespace collision_group
{
void bindings()
{
  class_<prx::collision_group_t, std::shared_ptr<prx::collision_group_t>>("collision_group", no_init)
      .def("__init__", make_constructor(&init_as_ptr<prx::collision_group_t, std::vector<prx::system_ptr_t>,
                                                     std::vector<std::shared_ptr<prx::movable_object_t>>>,
                                        default_call_policies(), (args("in_plants"), args("in_obstacles"))))
      .def("in_collision", &prx::collision_group_t::in_collision)
      // Comment to force ; to the next one
      ;
}
}  // namespace collision_group
}  // namespace collision_checking
}  // namespace simulation
}  // namespace pyprx