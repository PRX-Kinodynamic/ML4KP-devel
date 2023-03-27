#pragma once
#include <iostream>
#include <boost/python.hpp>
#include <boost/python/copy_const_reference.hpp>
#include <boost/python/return_value_policy.hpp>
#include "prx/utilities/general/dijkstra.hpp"
namespace pyprx
{
namespace utilities
{
namespace dijkstra
{
using prx::utilities::dijkstra_t;

void bindings()
{
  using NeighborsQuery = std::function<std::vector<std::size_t>(const std::size_t&)>;
  using DijkstraDistance = std::function<double(const std::size_t&, const std::size_t&)>;

  class_<NeighborsQuery>("neighbors_query")
      .def("__call__", &NeighborsQuery::operator())
      .def("wrap", &create_function<NeighborsQuery, std::vector<std::size_t>, const std::size_t&>)
      .staticmethod("wrap")
      // Comment to force ; to the next one
      ;
  class_<DijkstraDistance>("dijkstra_distance")
      .def("__call__", &DijkstraDistance::operator())
      .def("wrap", &create_function<DijkstraDistance, double, const std::size_t&, const std::size_t&>)
      .staticmethod("wrap")
      // Comment to force ; to the next one
      ;

  class_<dijkstra_t, boost::noncopyable>("dijkstra", no_init)
      .def("shortest_path", &dijkstra_t::shortest_path<std::size_t, NeighborsQuery, DijkstraDistance>)
      .staticmethod("shortest_path")
      // Comment to force ; to the next one
      ;
}
}  // namespace dijkstra
}  // namespace utilities
}  // namespace pyprx