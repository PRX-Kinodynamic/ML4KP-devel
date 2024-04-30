#include <iostream>
#include <boost/python.hpp>
#include "prx/utilities/geometry/basic_geoms/cylinder.hpp"

using namespace boost::python;

namespace pyprx
{
namespace utilities
{
namespace geometry
{
namespace basic_geoms
{
namespace cylinder
{
std::shared_ptr<prx::cylinder_t> create_cylinder(std::string object_name, double radius, double height,
                                                 const prx::transform_t pose, const std::string color)
{
  return std::make_shared<prx::cylinder_t>(object_name, radius, height, pose, color);
}

void bindings()
{
  class_<prx::cylinder_t, std::shared_ptr<prx::cylinder_t>, bases<prx::movable_object_t>>("cylinder", no_init)
      .def("create", &create_cylinder)
      .staticmethod("create");

  implicitly_convertible<std::shared_ptr<prx::cylinder_t>, std::shared_ptr<prx::movable_object_t>>();
}

}  // namespace cylinder
}  // namespace basic_geoms
}  // namespace geometry
}  // namespace utilities
}  // namespace pyprx