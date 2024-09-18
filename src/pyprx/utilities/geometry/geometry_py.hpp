#include <iostream>
#include <boost/python.hpp>
#include "prx/utilities/geometry/geometry.hpp"
#include "pyprx/utilities/geometry/movable_object_py.hpp"
#include "pyprx/utilities/geometry/basic_geoms/basic_geoms_py.hpp"

using namespace boost::python;

namespace pyprx
{
namespace utilities
{
namespace geometry
{

void bindings()
{
  class_<prx::geometry_t>("geometry", init<prx::geometry_type_t>());

  movable_object::bindings();
  basic_geoms::bindings();

  // register_ptr_to_python<std::shared_ptr<prx::movable_object_t>>();
}

}  // namespace geometry
}  // namespace utilities
}  // namespace pyprx