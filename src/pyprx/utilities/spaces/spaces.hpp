#include <iostream>
#include <boost/python.hpp>
#include "pyprx/utilities/spaces/space_py.hpp"
#include "pyprx/utilities/spaces/noisy_space_py.hpp"

namespace pyprx
{
namespace utilities
{
namespace spaces
{

void bindings()
{
  space::bindings();
  noisy_space::bindings();
}

}  // namespace spaces
}  // namespace utilities
}  // namespace pyprx