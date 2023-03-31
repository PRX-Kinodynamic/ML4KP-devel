#include <iostream>
#include <boost/python.hpp>
#include "prx/utilities/general/constants.hpp"

using namespace boost::python;

namespace pyprx
{
namespace utilities
{
namespace general
{
namespace constants
{

template <typename State, typename Steps, typename Bound>
static bool py_state_space_step(State& state, const Steps steps, const std::size_t& dimension, const Bound lower_bound,
                                const Bound upper_bound)
{
  return state_space_step(*state, steps, dimension, lower_bound, upper_bound);
}

void bindings()
{
  // Var("EPSILON", PRX_EPSILON);
  scope().attr("PRX_EPSILON") = PRX_EPSILON;
  scope().attr("PRX_INFINITY") = PRX_INFINITY;
  scope().attr("PRX_PI") = PRX_PI;
  scope().attr("lib_path") = prx::lib_path;
  scope().attr("models_path") = prx::models_path;
  scope().attr("input_path") = prx::input_path;
  scope().attr("js_path") = prx::js_path;
  scope().attr("out_path") = prx::out_path;

  enum_<prx::propagate_step>("propagate_step")
      .value("FIRST_STEP", prx::FIRST_STEP)
      .value("MIDDLE_STEP", prx::MIDDLE_STEP)
      .value("FINAL_STEP", prx::FINAL_STEP)
      .export_values();
  // TODO: Python can't handle function with same number of args, this has to happen on the c++ side
  // def("state_space_step", py_state_space_step<prx::space_point_t, double, std::vector<double>>);
  def("state_space_step", py_state_space_step<prx::space_point_t, std::vector<double>, std::vector<double>>);
  def("state_space_step", py_state_space_step<prx::space_point_t, double, std::vector<double>>);
}
}  // namespace constants
}  // namespace general
}  // namespace utilities
}  // namespace pyprx