#include <iostream>
#include <boost/python.hpp>
#include "pyprx/utilities/data_structures/abstract_edge_py.hpp"
#include "pyprx/utilities/data_structures/abstract_node_py.hpp"
#include "pyprx/utilities/data_structures/gnn_py.hpp"
#include "pyprx/utilities/data_structures/tree_py.hpp"
#include "pyprx/utilities/data_structures/delaunay_py.hpp"
namespace pyprx
{
namespace utilities
{
namespace data_structures
{

void bindings()
{
  gnn::bindings();
  abstract_edge::bindings();
  abstract_node::bindings();
  tree::bindings();
  delaunay::bindings();
}

}  // namespace data_structures
}  // namespace utilities
}  // namespace pyprx