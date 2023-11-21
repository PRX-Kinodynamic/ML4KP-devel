#include <iostream>
#include <boost/python.hpp>
#include "pyprx/planning/planners/planner_py.hpp"
// #include "pyprx/planning/planners/hyb_aorrt2_stride_py.hpp"
#include "pyprx/planning/planners/rrt_py.hpp"
#include "pyprx/planning/planners/sst_py.hpp"
#include "pyprx/planning/planners/dirt_py.hpp"

using namespace boost::python;
namespace pyprx
{
namespace planning
{
namespace planners
{
void bindings()
{
  planner::bindings();
  rrt::bindings();
  sst::bindings();
  dirt::bindings();
  // pyprx_planning_planners_hyb_aorrt2_stride_py();
}
}  // namespace planners
}  // namespace planning
}  // namespace pyprx