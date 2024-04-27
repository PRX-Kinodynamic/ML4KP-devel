#include <iostream>
#include <boost/python.hpp>

#include "pyprx/pyprx_utils.hpp"
#include "pyprx/stdlib_py.hpp"
#include "pyprx/simulation/simulation_py.hpp"
#include "pyprx/utilities/utilities_py.hpp"
#include "pyprx/planning/planning_py.hpp"
#include "pyprx/visualization/visualization_py.hpp"

BOOST_PYTHON_MODULE(PyML4KP)  // Name here must match the name of the final shared library, i.e. mantid.dll or
// mantid.so
{
  pyprx::stdlib::bindings();
  pyprx::utilities::bindings();
  pyprx::simulation::bindings();
  pyprx::planning::bindings();
  pyprx::visualization::bindings();
}