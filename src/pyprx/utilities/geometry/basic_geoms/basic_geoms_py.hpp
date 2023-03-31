#include <iostream>
#include <boost/python.hpp>
#include "pyprx/utilities/geometry/basic_geoms/box_py.hpp"
#include "pyprx/utilities/geometry/basic_geoms/cylinder_py.hpp"
#include "pyprx/utilities/geometry/basic_geoms/sphere_py.hpp"

using namespace boost::python;

namespace pyprx
{
namespace utilities
{
namespace geometry
{
namespace basic_geoms
{
void bindings()
{
  box::bindings();
  cylinder::bindings();
  sphere::bindings();
}

}  // namespace basic_geoms
}  // namespace geometry
}  // namespace utilities
}  // namespace pyprx