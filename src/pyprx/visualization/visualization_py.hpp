#include <iostream>
#include <boost/python.hpp>
#include "pyprx/visualization/three_js_group_py.hpp"

namespace pyprx
{
namespace visualization
{

void bindings()
{
  three_js_group::bindings();
}

}  // namespace visualization
}  // namespace pyprx