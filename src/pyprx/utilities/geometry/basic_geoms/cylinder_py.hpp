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

void bindings()
{
  // TODO
  // class_<prx::box_t, std::shared_ptr<prx::box_t>, bases<prx::movable_object_t>>("box", no_init)
  // 	.def("create_obstacle", &create_box).staticmethod("create_obstacle");
  // 	;

  // implicitly_convertible<std::shared_ptr<prx::box_t>, std::shared_ptr<prx::movable_object_t>>();
}

}  // namespace cylinder
}  // namespace basic_geoms
}  // namespace geometry
}  // namespace utilities
}  // namespace pyprx