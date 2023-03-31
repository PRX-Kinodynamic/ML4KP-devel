#pragma once
#include <iostream>
#include <boost/python.hpp>
#include "pyprx/utilities/spaces/spaces.hpp"
#include "pyprx/utilities/general/general_py.hpp"
#include "pyprx/utilities/geometry/geometry_py.hpp"
#include "pyprx/utilities/data_structures/data_structures_py.hpp"

namespace pyprx
{
namespace utilities
{

void bindings()
{
  spaces::bindings();
  general::bindings();
  geometry::bindings();
  data_structures::bindings();
}

}  // namespace utilities
}  // namespace pyprx