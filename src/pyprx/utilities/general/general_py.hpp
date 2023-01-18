#include <iostream>
#include <boost/python.hpp>
#include "pyprx/utilities/general/constants_py.hpp"
#include "pyprx/utilities/general/transforms_py.hpp"
#include "pyprx/utilities/general/random_py.hpp"
#include "pyprx/utilities/general/param_loader_py.hpp"
#include "pyprx/utilities/general/noise_py.hpp"

namespace pyprx
{
namespace utilities
{
namespace general
{

void bindings()
{
  constants::bindings();
  transforms::bindings();
  random::bindings();
  param_loader::bindings();
  noise::bindings();
}

}  // namespace general
}  // namespace utilities
}  // namespace pyprx