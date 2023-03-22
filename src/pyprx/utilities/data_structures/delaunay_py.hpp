#pragma once
#include <iostream>
#include <boost/python.hpp>
#include <boost/python/copy_const_reference.hpp>
#include <boost/python/return_value_policy.hpp>
#include "prx/utilities/data_structures/delaunay.hpp"
#include "pyprx/utilities/data_structures/gnn_py.hpp"
namespace pyprx
{
namespace utilities
{
namespace data_structures
{
namespace delaunay
{
using prx::utilities::delaunay_graph_t;
using prx::utilities::delaunay_node_t;
using delaunay_node_prx = delaunay_node_t<Eigen::Dynamic>;
using PyPoint = delaunay_node_prx::Point;
using delaunay_graph_prx = delaunay_graph_t<Eigen::Dynamic>;
using delaunay_sites_prx = delaunay_node_prx::Sites;
using delaunay_node_base = prx::proximity_node_t<PyPoint>;
using PyEigenMetric = prx::graph_nearest_neighbors_t<PyPoint, delaunay_node_prx>::Metric;

using shared_delaunay_node_prx = std::shared_ptr<delaunay_node_t<Eigen::Dynamic>>;
PRX_GETTER(delaunay_node_prx, sites)
PRX_SETTER(delaunay_node_prx, sites)
PRX_PTR_GETTER(shared_delaunay_node_prx, sites)
PRX_PTR_GETTER(shared_delaunay_node_prx, point)

void bindings()
{
  gnn::proximity_node_bindings<delaunay_node_base, PyPoint>("delaunay_proximity_node", init_as_ptr<delaunay_node_base>);
  gnn::gnn_bindings<PyPoint, delaunay_node_prx>(
      init_as_ptr<prx::graph_nearest_neighbors_t<PyPoint, delaunay_node_prx>, PyEigenMetric>);

  class_<delaunay_node_prx, bases<delaunay_node_base>, boost::noncopyable>("delaunay_node", no_init)
      .def("__init__", make_constructor(&init_as_ptr<delaunay_node_prx>, default_call_policies()))
      .def_readwrite("sites", &delaunay_node_prx::sites)
      // .add_property("sites", &get_delaunay_node_prx_sites<delaunay_sites_prx>)
      // .add_property("sites", &get_ptr_shared_delaunay_node_prx_sites<delaunay_sites_prx>)
      // .add_property("point", &get_ptr_shared_delaunay_node_prx_point<PyPoint>)
      //               boost::python::return_value_policy<boost::python::copy_const_reference>())
      // &set_delaunay_node_prx_sites<delaunay_sites_prx>);
      // Comment to force ; to the next one
      ;

  class_<delaunay_graph_prx, boost::noncopyable>("delaunay_graph", no_init)
      .def("__init__",
           make_constructor(&init_as_ptr<delaunay_graph_prx, delaunay_graph_prx::DelaunayMetric, const Eigen::Index>,
                            default_call_policies()))
      .def("qhull_to_delaunay", &delaunay_graph_prx::qhull_to_delaunay)
      .def("add_point", &delaunay_graph_prx::add_point<prx::space_point_t>)
      .def("get_gnn_nodes", &delaunay_graph_prx::get_gnn_nodes)
      .def("get_gnn", &delaunay_graph_prx::get_gnn)
      // Comment to force ; to the next one
      ;

  def("default_delaunay_metric", &delaunay_graph_prx::default_delaunay_metric);
  class_<delaunay_graph_prx::DelaunayMetric>("delaunay_metric")
      .def("__call__", &delaunay_graph_prx::DelaunayMetric::operator())
      .def("wrap", &create_function<delaunay_graph_prx::DelaunayMetric, double, const PyPoint&, const PyPoint&>)
      .staticmethod("wrap")
      // Comment to force ; to the next one
      ;

  // register_ptr_to_python<delaunay_node_base*>();
  // register_ptr_to_python<std::shared_ptr<delaunay_node_prx>>();
  register_ptr_with_check<delaunay_graph_prx*>();
  register_ptr_with_check<std::shared_ptr<delaunay_graph_prx>>();

  register_ptr_with_check<delaunay_node_prx*>();
  register_ptr_with_check<std::shared_ptr<delaunay_node_prx>>();

  register_ptr_with_check<PyPoint*>();
  register_ptr_with_check<std::shared_ptr<PyPoint>>();

  register_ptr_with_check<prx::graph_nearest_neighbors_t<PyPoint, delaunay_node_prx>*>();
  register_ptr_with_check<std::shared_ptr<prx::graph_nearest_neighbors_t<PyPoint, delaunay_node_prx>>>();

  container_wrapper<delaunay_graph_prx::GnnNodes>("GnnNodes");
  // container_wrapper<std::vector<delaunay_node_prx*>>("GnnNodePtrs");
  container_wrapper<delaunay_sites_prx>("DelaunaySites");
}
}  // namespace delaunay
}  // namespace data_structures
}  // namespace utilities
}  // namespace pyprx
