#include <boost/python.hpp>
#include <boost/python/copy_const_reference.hpp>
#include <boost/python/overloads.hpp>
#include <boost/python/return_value_policy.hpp>

#include "prx/simulation/playback/plan.hpp"

using namespace boost::python;

namespace pyprx
{
namespace simulation
{
namespace playback
{
namespace plan
{
prx::space_point_t (prx::plan_t::*plan_at_2)(double) const = &prx::plan_t::at;
void (prx::plan_t::*copy_onto_back_2)(prx::space_point_t, double) = &prx::plan_t::copy_onto_back;

void append_onto_back_1(prx::plan_t& plan, double time)
{
  plan.append_onto_back(time, false);
}
// void (prx::plan_t::*append_onto_back_2)(double, bool) = &prx::plan_t::append_onto_back;

// TODO: not working because of multiple definitions of append_onto_back
// BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(append_onto_back_overloads, append_onto_back, 1, 2)

void bindings()
{
  class_<prx::plan_step_t, std::shared_ptr<prx::plan_step_t>>("plan_step", no_init)
      .def("__init__", make_constructor(&init_as_ptr<prx::plan_step_t, prx::space_point_t, double>,
                                        default_call_policies(), (args("space"))))
      .def("copy_step", &prx::plan_step_t::copy_step)
      .add_property("control", &prx::plan_step_t::control)
      .add_property("duration", &prx::plan_step_t::duration)
      .def("__str__", &pyprx::stream_to_str<prx::plan_step_t>)
      // Comment to force ; to the next one
      ;

  class_<prx::plan_t, std::shared_ptr<prx::plan_t>>("plan", no_init)
      .def("__init__",
           make_constructor(&init_as_ptr<prx::plan_t, prx::space_t*>, default_call_policies(), (args("new_space"))))
      .def("__init__",
           make_constructor(&init_as_ptr<prx::plan_t, prx::plan_t>, default_call_policies(), (args("other"))))
      .def("__len__", &prx::plan_t::size)
      .def("size", &prx::plan_t::size)
      .def("duration", &prx::plan_t::duration)
      .def("__iter__", iterator<prx::plan_t>())
      .def("resize", &prx::plan_t::resize)
      .def(self += other<prx::plan_t>())
      .def("clear", &prx::plan_t::clear)
      .def("copy_to", &prx::plan_t::copy_to)
      .def("copy_onto_back", copy_onto_back_2)
      .def("copy_onto_front", &prx::plan_t::copy_onto_front)
      .def("append_onto_front", &prx::plan_t::append_onto_front)
      .def("append_onto_back", append_onto_back_1)
      .def("extend_last_control", &prx::plan_t::extend_last_control)
      .def("pop_front", &prx::plan_t::pop_front)
      .def("pop_back", &prx::plan_t::pop_back)
      .def("print", &prx::plan_t::print)
      .def("to_file", &prx::plan_t::to_file)
      .def("from_file", &prx::plan_t::from_file)
      .def("expand", &prx::plan_t::expand)
      .def("compress", &prx::plan_t::compress)
      .def("__str__", &pyprx::stream_to_str<prx::plan_t>)
      .def("__getitem__", &pyprx::wrapper_subscript_oper_to_get_item<prx::plan_t, prx::plan_step_t>,
           return_value_policy<copy_non_const_reference>())
      .def("__setitem__", &pyprx::wrapper_subscript_oper_to_set_item<prx::plan_t, prx::plan_step_t>)
      // Comment to force ; to the next one
      ;
}

}  // namespace plan
}  // namespace playback
}  // namespace simulation
}  // namespace pyprx