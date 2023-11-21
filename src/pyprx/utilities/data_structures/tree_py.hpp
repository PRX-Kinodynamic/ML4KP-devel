#include "prx/utilities/data_structures/tree.hpp"

namespace pyprx
{
namespace utilities
{
namespace data_structures
{
namespace tree
{

using prx::tree_t;

PRX_SETTER(tree_t, vertex_id_counter)
PRX_GETTER(tree_t, vertex_id_counter)

void bindings()
{
  class_<prx::tree_node_t, std::shared_ptr<prx::tree_node_t>, bases<prx::abstract_node_t>>("tree_node", init<>())
      .def("__init__", make_constructor(&init_as_ptr<prx::tree_node_t>, default_call_policies()))
      .def("get_parent", &prx::tree_node_t::get_parent)
      .def("get_index", &prx::tree_node_t::get_index)
      .def("get_parent_edge", &prx::tree_node_t::get_parent_edge)
      .def("get_children", &prx::tree_node_t::get_children, return_internal_reference<>())
      // Comment to force ; to the next one
      ;

  class_<prx::tree_edge_t, std::shared_ptr<prx::tree_edge_t>, bases<prx::abstract_edge_t>, boost::noncopyable>(
      "tree_edge", no_init)
      .def("__init__", make_constructor(&init_as_ptr<prx::tree_edge_t>, default_call_policies()))
      .def("get_index", &prx::tree_edge_t::get_index)
      .def("get_source", &prx::tree_edge_t::get_source)
      .def("get_target", &prx::tree_edge_t::get_target)
      // Comment to force ; to the next one
      ;

  class_<tree_t, std::shared_ptr<tree_t>>("tree", init<>())
      .def("allocate_memory", &tree_t::allocate_memory<prx::tree_node_t, prx::tree_edge_t>)
      .def("add_vertex", &tree_t::add_vertex<prx::tree_node_t, prx::tree_edge_t>)
      .def("get_vertex_as", &tree_t::get_vertex_as<prx::tree_node_t>)
      .def("get_edge_as", &tree_t::get_edge_as<prx::tree_edge_t>)
      .def("num_vertices", &tree_t::num_vertices)
      .def("num_edges", &tree_t::num_edges)
      .def("is_leaf", &tree_t::is_leaf)
      .def("edge", &tree_t::edge)
      .def("add_edge", &tree_t::add_edge)
      .def("get_depth", &tree_t::get_depth)
      .def("remove_vertex", &tree_t::remove_vertex)
      .def("purge", &tree_t::purge)
      .def("clear", &tree_t::clear)
      .def("transplant", &tree_t::transplant)
      .add_property("vertex_id_counter", &get_tree_t_vertex_id_counter<unsigned>,
                    &set_tree_t_vertex_id_counter<unsigned>)
      // Comment to force ; to the next one
      ;
}

}  // namespace tree
}  // namespace data_structures
}  // namespace utilities
}  // namespace pyprx
