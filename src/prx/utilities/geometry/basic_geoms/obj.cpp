#include "prx/utilities/geometry/basic_geoms/obj.hpp"

namespace prx
{
obj_t::obj_t(const std::string& object_name, const std::string obj_fname, const transform_t& pose)
  : movable_object_t(object_name)
{
  geometries["body"] = std::make_shared<geometry_t>(geometry_type_t::OBJ);
  geometries["body"]->initialize_obj_geometry(obj_fname );
  geometries["body"]->generate_collision_geometry();
  configurations["body"] = std::make_shared<transform_t>();
  *configurations["body"] = pose;
}
}  // namespace prx