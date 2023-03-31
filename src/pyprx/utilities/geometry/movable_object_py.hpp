#include <iostream>
#include <boost/python.hpp>
#include "prx/utilities/geometry/movable_object.hpp"

using namespace boost::python;

namespace pyprx
{
namespace utilities
{
namespace geometry
{
namespace movable_object
{

std::vector<std::pair<std::string, std::shared_ptr<prx::geometry_t>>> get_geometries_shared(prx::movable_object_t obj)
{
  std::vector<std::pair<std::string, std::shared_ptr<prx::geometry_t>>> pairs;

  auto gg = obj.get_geometries();
  for (auto p : gg)
  {
    pairs.push_back(std::make_pair(p.first, p.second.lock()));
  }
  return pairs;
}

std::vector<std::pair<std::string, std::shared_ptr<prx::transform_t>>>
get_configurations_shared(prx::movable_object_t obj)
{
  std::vector<std::pair<std::string, std::shared_ptr<prx::transform_t>>> pairs;

  auto gg = obj.get_configurations();
  for (auto p : gg)
  {
    pairs.push_back(std::make_pair(p.first, p.second.lock()));
  }
  return pairs;
}

void bindings()
{
  class_<prx::movable_object_t, std::shared_ptr<prx::movable_object_t>>("movable_object", init<std::string>())
      .def("get_geometries", &get_geometries_shared)
      .def("get_configurations", &get_configurations_shared);

  class_<std::pair<std::string, std::shared_ptr<prx::geometry_t>>>("movable_object_geometry")
      .def_readwrite("name", &std::pair<std::string, std::shared_ptr<prx::geometry_t>>::first)
      .def_readwrite("geometry", &std::pair<std::string, std::shared_ptr<prx::geometry_t>>::second);

  class_<std::pair<std::string, std::shared_ptr<prx::transform_t>>>("movable_object_transform")
      .def_readwrite("name", &std::pair<std::string, std::shared_ptr<prx::transform_t>>::first)
      .def_readwrite("transform", &std::pair<std::string, std::shared_ptr<prx::transform_t>>::second);

  PRX_ITERABLE_WRAPPER(std::vector<std::shared_ptr<prx::movable_object_t>>, "vector_of_movable_object")
  iterable_converter().from_python<std::vector<std::shared_ptr<prx::movable_object_t>>>();

  PRX_ITERABLE_WRAPPER_NONSTR(SINGLE_ARG(std::vector<std::pair<std::string, std::shared_ptr<prx::geometry_t>>>),
                              "vector_of_movable_object_geometry")
  PRX_ITERABLE_WRAPPER_NONSTR(SINGLE_ARG(std::vector<std::pair<std::string, std::shared_ptr<prx::transform_t>>>),
                              "vector_of_movable_object_transform")
}

}  // namespace movable_object
}  // namespace geometry
}  // namespace utilities
}  // namespace pyprx