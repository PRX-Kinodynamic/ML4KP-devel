#include <iostream>
#include <boost/python.hpp>
#include "prx/utilities/geometry/basic_geoms/obj.hpp"

using namespace boost::python;

namespace pyprx
{
namespace utilities
{
namespace geometry
{
namespace basic_geoms
{
namespace obj
{
std::shared_ptr<prx::obj_t> create_obj(std::string object_name, const std::string obj_fname,
                                       const prx::transform_t pose)
{
  return std::make_shared<prx::obj_t>(object_name, obj_fname, pose);
}

void bindings()
{
  class_<prx::obj_t, std::shared_ptr<prx::obj_t>, bases<prx::movable_object_t>>("obj", no_init)
      .def("create", &create_obj)
      .staticmethod("create");

  implicitly_convertible<std::shared_ptr<prx::obj_t>, std::shared_ptr<prx::movable_object_t>>();
}

}  // namespace obj
}  // namespace basic_geoms
}  // namespace geometry
}  // namespace utilities
}  // namespace pyprx