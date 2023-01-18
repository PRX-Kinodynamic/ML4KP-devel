#include <iostream>
#include <boost/python.hpp>
#include "pyprx/planning/world_model_py.hpp"
#include "pyprx/planning/condition_check_py.hpp"
#include "pyprx/planning/planners/planners_py.hpp"
#include "pyprx/planning/planner_statistics_py.hpp"
#include "pyprx/planning/planner_functions/planner_functions_py.hpp"
#include "pyprx/planning/noisy_world_model_py.hpp"

using namespace boost::python;

namespace pyprx
{
namespace planning
{
void bindings()
{
  condition_check::bindings();
  planner_functions::bindings();
  planner_statistics::bindings();
  planners::bindings();
  world_model::bindings();
  noisy_world_model::bindings();
}
}  // namespace planning
}  // namespace pyprx