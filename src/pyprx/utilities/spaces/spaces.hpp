#include <iostream>
#include <boost/python.hpp>
#include "pyprx/utilities/spaces/space_py.hpp"
#include "pyprx/utilities/spaces/noisy_space_py.hpp"

void pyprx_utilities_spaces()
{
  pyprx_utilities_spaces_space();
  pyprx_utilities_spaces_noisy_space();
}