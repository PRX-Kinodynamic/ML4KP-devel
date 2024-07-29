#include <iostream>
#include <boost/python.hpp>

#include "pyprx/planning/planners/planners_py.hpp"
#include "pyprx/planning/planner_functions/planner_functions_py.hpp"

#include "pyprx/planning/planner_statistics_py.hpp"
#include "pyprx/planning/world_model_py.hpp"

using namespace boost::python;

namespace pyprx
{
namespace planning
{
void bindings()
{
  planner_functions::bindings();
  planner_statistics::bindings();
  planners::bindings();
  world_model::bindings();
}
}  // namespace planning
}  // namespace pyprx