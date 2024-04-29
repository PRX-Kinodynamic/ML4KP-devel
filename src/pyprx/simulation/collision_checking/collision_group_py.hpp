#include "prx/simulation/collision_checking/collision_group.hpp"

namespace pyprx
{
namespace simulation
{
namespace collision_checking
{
namespace collision_group
{

using ClosestPoints = std::pair<Eigen::Vector3d, Eigen::Vector3d>;
using VectorClosestPoints = std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>;

void bindings()
{
  class_<ClosestPoints, std::shared_ptr<ClosestPoints>>("pair_of_points", no_init)
      .def_readwrite("first", &ClosestPoints::first)
      .def_readwrite("second", &ClosestPoints::second)
      // Comment to force ; to the next one
      ;
  class_<prx::collision_group_t::pqp_distance_t, std::shared_ptr<prx::collision_group_t::pqp_distance_t>>(
      "pqp_distance", no_init)
      .def_readwrite("distances", &prx::collision_group_t::pqp_distance_t::distances)
      .def_readwrite("closest_points", &prx::collision_group_t::pqp_distance_t::closest_points)
      // Comment to force ; to the next one
      ;

  class_<prx::collision_group_t, std::shared_ptr<prx::collision_group_t>>("collision_group", no_init)
      .def("__init__", make_constructor(&init_as_ptr<prx::collision_group_t, std::vector<prx::system_ptr_t>,
                                                     std::vector<std::shared_ptr<prx::movable_object_t>>>,
                                        default_call_policies(), (args("in_plants"), args("in_obstacles"))))
      .def("in_collision", &prx::collision_group_t::in_collision)
      .def("add_new_obstacle", &prx::collision_group_t::add_new_obstacle)
      .def("get_distances", &prx::collision_group_t::get_distances)
      // Comment to force ; to the next one
      ;
  class_<VectorClosestPoints>("PairsOfPoints", init<>())  // no-lint
      .def(vector_indexing_suite<VectorClosestPoints>())
      // Comment to force ; to the next one
      ;
  // vector_wrapper<std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>>("PairsOfPoints");
}
}  // namespace collision_group
}  // namespace collision_checking
}  // namespace simulation
}  // namespace pyprx