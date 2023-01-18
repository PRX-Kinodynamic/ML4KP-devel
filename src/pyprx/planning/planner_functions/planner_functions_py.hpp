#include <iostream>
#include <boost/python.hpp>
#include "prx/planning/planner_functions/planner_functions.hpp"

using namespace boost::python;

namespace pyprx
{
namespace planning
{
namespace planner_functions
{
prx::heuristic_function_t init_heuristic_function()
{
  prx::heuristic_function_t default_h = [](const prx::space_point_t& s1, const prx::space_point_t& s2) {
    return sqrt((s1->at(0) - s2->at(0)) * (s1->at(0) - s2->at(0)) + (s1->at(1) - s2->at(1)) * (s1->at(1) - s2->at(1)));
  };
  // return boost::python::make_function(default_df);
  return default_h;
}

void bindings()
{
  def("default_valid_state", &prx::default_valid_state);
  class_<prx::valid_state_t>("valid_state")
      .def("__call__", &prx::valid_state_t::operator())
      .def("wrap", &create_function<prx::valid_state_t, bool, prx::space_point_t>)
      .staticmethod("wrap")
      // Comment to force ; to the next one
      ;

  def("default_valid_trajectory", &prx::default_valid_trajectory);
  class_<prx::valid_trajectory_t>("valid_trajectory")
      .def("__call__", &prx::valid_trajectory_t::operator())
      .def("set_f", &create_function<prx::valid_trajectory_t, bool, prx::trajectory_t>)
      .staticmethod("set_f")
      .def("wrap", &create_function<prx::valid_trajectory_t, bool, prx::trajectory_t>)
      .staticmethod("wrap");

  class_<prx::sample_state_t>("sample_state")
      .def("__call__", &prx::sample_state_t::operator())
      .def("wrap", &create_function<prx::sample_state_t, void, prx::space_point_t&>)
      .staticmethod("wrap");

  class_<prx::sample_plan_t>("sample_plan")
      .def("__call__", &prx::sample_plan_t::operator())
      .def("wrap", &create_function<prx::sample_plan_t, void, prx::plan_t, prx::space_point_t>)
      .staticmethod("wrap");

  class_<prx::valid_stop_t>("valid_stop")
      .def("__call__", &prx::valid_stop_t::operator())
      .def("wrap", &create_function<prx::valid_stop_t, bool, prx::space_point_t, prx::plan_t*, prx::trajectory_t*>)
      .staticmethod("wrap");

  class_<prx::propagate_t>("propagate")
      .def("__call__", &prx::propagate_t::operator())
      .def("wrap", &create_function<prx::propagate_t, void, prx::space_point_t, prx::plan_t&, prx::trajectory_t&>)
      .staticmethod("wrap");

  class_<prx::expand_t>("expand")
      .def("__call__", &prx::expand_t::operator())
      .def("wrap", &create_function<prx::expand_t, void, prx::space_point_t&, std::vector<prx::plan_t*>&,
                                    std::vector<prx::trajectory_t*>&, int, bool>)
      .staticmethod("wrap");

  def("create_default_goal_check", &prx::create_default_goal_check);
}
}  // namespace planner_functions
}  // namespace planning
}  // namespace pyprx