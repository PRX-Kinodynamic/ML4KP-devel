#pragma once
#include "prx/utilities/data_structures/gnn.hpp"

namespace pyprx
{
namespace utilities
{
namespace data_structures
{
namespace gnn
{
using prx::proximity_node_t;
using prx::space_point_t;

PRX_SETTER(proximity_node_t, added_index)
PRX_GETTER(proximity_node_t, added_index)

std::vector<long unsigned> get_neighbors_wrapper(std::shared_ptr<prx::proximity_node_t> pn, long unsigned nr_neigh)
{
  std::vector<long unsigned> v;
  long unsigned* neighbors = pn->get_neighbors(&nr_neigh);
  for (int i = 0; i < nr_neigh; i++)
  {
    v.push_back(neighbors[i]);
  }
  return v;
}

template <typename ProximityNode, typename Point, typename InitPtr>
void proximity_node_bindings(const std::string name, InitPtr init_ptr)
{
  class_<ProximityNode, boost::noncopyable>(name.c_str(), no_init)
      .def("__init__", make_constructor(init_ptr, default_call_policies()))
      .def("get_prox_index", &ProximityNode::get_prox_index)
      .def("set_index", &ProximityNode::set_index)
      .def("get_neighbors", get_neighbors_wrapper)
      .def("add_neighbor", &ProximityNode::add_neighbor)
      .def("delete_neighbor", &ProximityNode::delete_neighbor)
      .def("replace_neighbor", &ProximityNode::replace_neighbor)
      .def("remove_all_neighbors", &ProximityNode::remove_all_neighbors)
      .add_property("added_index", &get_proximity_node_t_added_index<long unsigned>,
                    &set_proximity_node_t_added_index<long unsigned>)
      // .add_property("point", make_function(get_proximity_node_t_point<Point, Point>, default_call_policies()),
      //               &set_proximity_node_t_point<Point, Point>)
      // .def_readwrite("point", &ProximityNode::point);
      // .add_property("point", &get_proximity_node_point<ProximityNode, Point&>,
      //               return_value_policy<copy_const_reference>())
      // .add_property("point", &get_ptr_proximity_node_point<std::shared_ptr<ProximityNode>, Point>,
      // return_value_policy<return_by_value>())
      // .def_readonly("point", &ProximityNode::point)
      // .add_property("point", make_getter(&ProximityNode::point, return_value_policy<return_const>()))
      // Comment to force ; to the next one
      ;
  register_ptr_with_check<ProximityNode*>();
  register_ptr_to_python<std::shared_ptr<ProximityNode>>();

  register_ptr_with_check<Point*>();
  register_ptr_with_check<std::shared_ptr<Point>>();
}

template <typename Point, typename Node, typename InitPtr>
void gnn_bindings(InitPtr init_ptr)
{
  using GNN = prx::graph_nearest_neighbors_t;
  class_<GNN, std::shared_ptr<GNN>, boost::noncopyable>("graph_nearest_neighbors", no_init)
      .def("__init__", make_constructor(init_ptr, default_call_policies(), (arg("distance_function"))))
      .def("node_distance", &GNN::node_distance)
      .def("add_node", &GNN::add_node)
      .def("remove_node", &GNN::remove_node)
      .def("clear", &GNN::clear)
      .def("get_nr_nodes", &GNN::get_nr_nodes)
      .def("single_query", &GNN::single_query, return_internal_reference<>())
      .def("multi_query", &GNN::multi_query)
      .def("radius_and_closest_query", &GNN::radius_and_closest_query)
      // Comment to force ; to the next one
      ;
}

void bindings()
{
  using ProximityNode = prx::proximity_node_t;
  proximity_node_bindings<ProximityNode, prx::space_point_t>("proximity_node", init_as_ptr<ProximityNode>);
  gnn_bindings<prx::space_point_t, ProximityNode>(
      init_as_ptr<prx::graph_nearest_neighbors_t, prx::distance_function_t>);

  // using PyMetric = prx::graph_nearest_neighbors_t<boost::python::object, proximity_node_py>::Metric;
  // proximity_node_bindings<proximity_node_py, boost::python::object>("proximity_node_pyobj",
  //                                                                   init_as_ptr<proximity_node_py, PyObject*>);
}

}  // namespace gnn
}  // namespace data_structures
}  // namespace utilities
}  // namespace pyprx