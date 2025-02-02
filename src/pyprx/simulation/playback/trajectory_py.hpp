#include <iostream>
#include <fstream>
#include <boost/python.hpp>
#include "prx/simulation/playback/trajectory.hpp"
namespace pyprx
{
namespace simulation
{
namespace playback
{
namespace trajectory
{

prx::space_point_t at_1(prx::trajectory_t traj, int i)
{
  return traj.at((unsigned)i);
}
// prx::space_point_t (prx::trajectory_t::*at_2)(double) const = &prx::trajectory_t::at;
prx::space_point_t (prx::trajectory_t::*at_2)(double, bool) const = &prx::trajectory_t::at;

void (prx::trajectory_t::*copy_onto_back_1)(prx::space_point_t) = &prx::trajectory_t::copy_onto_back;
void (prx::trajectory_t::*copy_onto_back_2)(const prx::space_t*) = &prx::trajectory_t::copy_onto_back;

// void (prx::trajectory_t::*print_0)(3)  = &prx::trajectory_t::print;
// void (prx::trajectory_t::*print_1)(unsigned) = &prx::trajectory_t::print;

void to_file(prx::trajectory_t& traj, std::string filename, std::string mode = "w")
{
  std::ios_base::openmode _mode;
  if (mode == "w" || mode == "W")
  {
    _mode = std::ofstream::trunc;
  }
  else if (mode == "a" || mode == "A")
  {
    _mode = std::ofstream::app;
  }
  prx_throw("NOT IMPLEMENTED");
  // traj.to_file(filename, _mode);
}
BOOST_PYTHON_FUNCTION_OVERLOADS(to_file_overloads, to_file, 2, 3);

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(trajectory_print_overloads, print, 0, 1)

void bindings()
{
  class_<prx::trajectory_t, std::shared_ptr<prx::trajectory_t>>("trajectory", no_init)
      .def("__init__",
           make_constructor(&init_as_ptr<prx::trajectory_t, prx::space_t*>, default_call_policies(), (args("space"))))
      .def("__init__", make_constructor(&init_as_ptr<prx::trajectory_t, prx::trajectory_t>, default_call_policies(),
                                        (args("traj"))))
      .def("__len__", &prx::trajectory_t::size)
      .def("__getitem__", at_1)
      .def("interpolate", at_2)
      .def("__iter__", iterator<prx::trajectory_t>())
      .def("front", &prx::trajectory_t::front)
      .def("back", &prx::trajectory_t::back)
      .def("resize", &prx::trajectory_t::resize)
      .def("copy", &prx::trajectory_t::copy)
      .def("clear", &prx::trajectory_t::clear)
      .def("print", &prx::trajectory_t::print, trajectory_print_overloads())
      .def("__str__", &pyprx::stream_to_str<prx::trajectory_t>)
      // .def<std::string (prx::trajectory_t::*)(unsigned)>("print", &prx::trajectory_t::print)
      .def("copy_onto_back", copy_onto_back_1)
      .def("copy_onto_back", copy_onto_back_2)
      // .def("to_file", to_file, to_file_overloads())
      // .def("from_file", &prx::trajectory_t::from_file)

      // .def("copy_onto_back", copy_onto_back_2)
      .def(self += other<prx::trajectory_t>())
      .def(self == other<prx::trajectory_t>())
      .def(self != other<prx::trajectory_t>());

  PRX_ITERABLE_WRAPPER(std::vector<prx::trajectory_t>, "vector_of_trajectories")
}

}  // namespace trajectory
}  // namespace playback
}  // namespace simulation
}  // namespace pyprx