#include <iostream>
#include <boost/python.hpp>
#include "prx/utilities/geometry/basic_geoms/sphere.hpp"

using namespace boost::python;

namespace pyprx
{
namespace utilities
{
namespace geometry
{
namespace basic_geoms
{
namespace sphere
{
std::shared_ptr<prx::sphere_t> create_sphere(std::string object_name, double rad, const prx::transform_t pose,
                                             const std::string color)
{
  return std::make_shared<prx::sphere_t>(object_name, rad, pose, color);
}

void bindings()
{
  class_<prx::sphere_t, std::shared_ptr<prx::sphere_t>, bases<prx::movable_object_t>>("sphere", no_init)
      .def("create", &create_sphere)
      .staticmethod("create");

  implicitly_convertible<std::shared_ptr<prx::sphere_t>, std::shared_ptr<prx::movable_object_t>>();
}

}  // namespace sphere
}  // namespace basic_geoms
}  // namespace geometry
}  // namespace utilities
}  // namespace pyprx