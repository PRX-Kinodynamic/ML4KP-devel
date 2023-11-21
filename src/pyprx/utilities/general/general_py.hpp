#include <iostream>
#include <boost/python.hpp>
#include "pyprx/utilities/general/condition_check_py.hpp"
#include "pyprx/utilities/general/constants_py.hpp"
#include "pyprx/utilities/general/transforms_py.hpp"
#include "pyprx/utilities/general/random_py.hpp"
#include "pyprx/utilities/general/param_loader_py.hpp"
#include "pyprx/utilities/general/noise_py.hpp"
#include "pyprx/utilities/general/prx_assert_py.hpp"

namespace pyprx
{
namespace utilities
{
namespace general
{

void bindings()
{
  constants::bindings();
  noise::bindings();
  param_loader::bindings();
  prx_assert::bindings();
  random::bindings();
  transforms::bindings();
  condition_check::bindings();
}

}  // namespace general
}  // namespace utilities
}  // namespace pyprx