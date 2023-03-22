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

template <typename T>
using shared_proximity_node = std::shared_ptr<prx::proximity_node_t<T>>;
PRX_SETTER_TEMPLATE(proximity_node_t, point)
PRX_GETTER_TEMPLATE(proximity_node_t, point)
PRX_SETTER_TEMPLATE(proximity_node_t, added_index)
PRX_GETTER_TEMPLATE(proximity_node_t, added_index)

template <typename Object, typename ReturnType>
ReturnType get_proximity_node_point(Object& o)
{
  return o.point;
}
template <typename Object, typename ReturnType>
ReturnType get_ptr_proximity_node_point(Object& o)
{
  return o->point;
}

template <typename Point>
std::vector<long unsigned> get_neighbors_wrapper(std::shared_ptr<prx::proximity_node_t<Point>> pn,
                                                 long unsigned nr_neigh)
{
  std::vector<long unsigned> v;
  long unsigned* neighbors = pn->get_neighbors(&nr_neigh);
  for (int i = 0; i < nr_neigh; i++)
  {
    v.push_back(neighbors[i]);
  }
  return v;
}

class proximity_node_py : public proximity_node_t<boost::python::object>
{
public:
  proximity_node_py(PyObject* self) : fSelf(self)
  {
  }
  virtual ~proximity_node_py()
  {
  }

private:
  PyObject* fSelf;
};

template <typename ProximityNode, typename Point, typename InitPtr>
void proximity_node_bindings(const std::string name, InitPtr init_ptr)
{
  class_<ProximityNode, boost::noncopyable>(name.c_str(), no_init)
      .def("__init__", make_constructor(init_ptr, default_call_policies()))
      .def("get_prox_index", &ProximityNode::get_prox_index)
      .def("set_index", &ProximityNode::set_index)
      .def("get_neighbors", get_neighbors_wrapper<Point>)
      .def("add_neighbor", &ProximityNode::add_neighbor)
      .def("delete_neighbor", &ProximityNode::delete_neighbor)
      .def("replace_neighbor", &ProximityNode::replace_neighbor)
      .def("remove_all_neighbors", &ProximityNode::remove_all_neighbors)
      .add_property("added_index", &get_proximity_node_t_added_index<long unsigned, Point>,
                    &set_proximity_node_t_added_index<long unsigned, Point>)
      // .add_property("point", make_function(get_proximity_node_t_point<Point, Point>, default_call_policies()),
      //               &set_proximity_node_t_point<Point, Point>)
      .def_readwrite("point", &ProximityNode::point);
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
  class_<prx::graph_nearest_neighbors_t<Point, Node>, std::shared_ptr<prx::graph_nearest_neighbors_t<Point, Node>>,
         boost::noncopyable>("graph_nearest_neighbors", no_init)
      .def("__init__", make_constructor(init_ptr, default_call_policies(), (arg("distance_function"))))
      .def("node_distance", &prx::graph_nearest_neighbors_t<Point, Node>::node_distance)
      .def("add_node", &prx::graph_nearest_neighbors_t<Point, Node>::add_node)
      .def("remove_node", &prx::graph_nearest_neighbors_t<Point, Node>::remove_node)
      .def("clear", &prx::graph_nearest_neighbors_t<Point, Node>::clear)
      .def("get_nr_nodes", &prx::graph_nearest_neighbors_t<Point, Node>::get_nr_nodes)
      .def("single_query", &prx::graph_nearest_neighbors_t<Point, Node>::single_query, return_internal_reference<>())
      .def("multi_query", &prx::graph_nearest_neighbors_t<Point, Node>::multi_query)
      .def("radius_and_closest_query",
           &prx::graph_nearest_neighbors_t<Point, Node>::template radius_and_closest_query<Point>)
      .def("radius_and_closest_query",
           &prx::graph_nearest_neighbors_t<Point, Node>::template radius_and_closest_query<std::vector<double>>)
      .def("radius_and_closest_query",
           &prx::graph_nearest_neighbors_t<Point, Node>::template radius_and_closest_query<Eigen::VectorXd>)
      .def("radius_and_closest_query",
           &prx::graph_nearest_neighbors_t<Point, Node>::template radius_and_closest_query<prx::space_point_t>)
      // Comment to force ; to the next one
      ;
}

void bindings()
{
  // boost::python::object
  proximity_node_bindings<prx::proximity_node_t<prx::space_point_t>, prx::space_point_t>(
      "proximity_node", init_as_ptr<prx::proximity_node_t<prx::space_point_t>>);
  gnn_bindings<prx::space_point_t, prx::proximity_node_t<prx::space_point_t>>(
      init_as_ptr<prx::graph_nearest_neighbors_t<prx::space_point_t, prx::proximity_node_t<prx::space_point_t>>,
                  prx::distance_function_t>);

  using PyMetric = prx::graph_nearest_neighbors_t<boost::python::object, proximity_node_py>::Metric;
  proximity_node_bindings<proximity_node_py, boost::python::object>("proximity_node_pyobj",
                                                                    init_as_ptr<proximity_node_py, PyObject*>);

  // using PyEigenPoint = Eigen::Vector<double, Eigen::Dynamic>;
  // using PyEigenNode = prx::proximity_node_t<PyEigenPoint>;
  // using PyEigenMetric = prx::graph_nearest_neighbors_t<PyEigenPoint, PyEigenNode>::Metric;
  // proximity_node_bindings<PyEigenNode, PyEigenPoint>(init_as_ptr<PyEigenNode>);
  // gnn_bindings<PyEigenPoint, PyEigenNode>(
  //     init_as_ptr<prx::graph_nearest_neighbors_t<PyEigenPoint, PyEigenNode>, PyEigenMetric>);

  // register_ptr_to_python<std::shared_ptr<PyEigenNode>>();
  // register_ptr_to_python<std::shared_ptr<prx::graph_nearest_neighbors_t<PyEigenPoint, PyEigenNode>>>();

  // gnn_bindings<boost::python::object, proximity_node_py>(
  //     init_as_ptr<prx::graph_nearest_neighbors_t<boost::python::object, proximity_node_py>, PyMetric>);
}

}  // namespace gnn
}  // namespace data_structures
}  // namespace utilities
}  // namespace pyprx