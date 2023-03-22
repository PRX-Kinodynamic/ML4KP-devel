#include <iostream>
#include <boost/python.hpp>
#include "prx/simulation/multivalued_map/time_map.hpp"

namespace pyprx
{
namespace simulation
{
namespace multivalued_map
{
namespace time_map
{
using prx::simulation::time_map_t;
PRX_SETTER(time_map_t, x_goal)
PRX_GETTER(time_map_t, x_goal)

PRX_SETTER(time_map_t, u_goal)
PRX_GETTER(time_map_t, u_goal)

std::mutex mutex_start_state;

template <typename T, std::enable_if_t<is_pyobject<T>{}, bool> = true>
inline prx::space_point_t to_cppobject(const std::shared_ptr<time_map_t> tm, T& object)
{
  static prx::space_point_t _point_0;
  static prx::space_point_t _point_1;
  if (_point_0 == nullptr)
    _point_0 = tm->get_state_space()->make_point();
  if (_point_1 == nullptr)
    _point_1 = tm->get_state_space()->make_point();
  if (mutex_start_state.try_lock())
  {
    return _point_0;
  }
  else
  {
    return _point_1;
  }
}
template <typename T, std::enable_if_t<!is_pyobject<T>{}, bool> = true>
inline T& to_cppobject(const std::shared_ptr<time_map_t> tm, T& object)
{
  return object;
}

template <typename T1, typename T2, std::enable_if_t<std::is_same<T1, T2>::value, bool> = true>
inline void to_pyobject(T1& t1, T2& t2)
{
  // return t1;
}

template <typename T1, typename T2, std::enable_if_t<!std::is_same<T1, T2>::value, bool> = true>
void to_pyobject(const T1& t1, T2& t2)
{
  prx_assert(len(t2) == t1->size(), "PyObject lenght does not match point size: " << len(t2) << " vs " << t1->size());
  container_to_pyobject(t2, *t1);
  mutex_start_state.unlock();
}

template <typename StartState, typename EndState>
void timemap_call_pyobject(const std::shared_ptr<time_map_t> tm, StartState& start_state, EndState& end_state)
{
  // static pyspace_snapshot_t _start_pt;
  // static pyspace_snapshot_t _end_pt;
  // _start_pt.pylist = &start_state;
  // _end_pt.pylist = &end_state;
  auto start_point = to_cppobject(tm, start_state);
  auto end_point = to_cppobject(tm, end_state);
  tm->operator()(start_point, end_point);
  to_pyobject(start_point, start_state);
  to_pyobject(end_point, end_state);
  // _start_pt.pylist = nullptr;
  // _end_pt.pylist = nullptr;
}

void bindings()
{
  class_<time_map_t, std::shared_ptr<time_map_t>>("time_map", no_init)
      .def("__init__",
           make_constructor(&init_as_ptr<time_map_t, const std::string, const prx::system_ptr_t,
                                         const std::shared_ptr<prx::system_group_t>>,
                            default_call_policies(), (arg("system_name"), arg("system_ptr"), arg("system_group"))))
      .def("set_duration", &time_map_t::set_duration)
      .def("__call__", timemap_call_pyobject<boost::python::list, boost::python::list>)
      .def("__call__", timemap_call_pyobject<boost::python::list, prx::space_point_t>)
      .def("__call__", timemap_call_pyobject<prx::space_point_t, boost::python::list>)
      .def("__call__", &time_map_t::operator()<prx::space_point_t, prx::trajectory_t>)
      .def("__call__", &time_map_t::operator()<prx::space_point_t, prx::space_point_t>)
      .add_property("x_goal", &get_time_map_t_x_goal<prx::space_point_t>, &set_time_map_t_x_goal<prx::space_point_t>)
      .add_property("u_goal", &get_time_map_t_u_goal<prx::space_point_t>, &set_time_map_t_u_goal<prx::space_point_t>)
      // Comment to force ; to the next one
      ;
}
}  // namespace time_map
}  // namespace multivalued_map
}  // namespace simulation
}  // namespace pyprx