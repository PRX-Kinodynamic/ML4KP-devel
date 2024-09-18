#include "prx/simulation/controllers/lqr.hpp"

namespace pyprx
{
namespace simulation
{
namespace controllers
{
namespace lqr
{
template <class... Types>
void set_goal(std::shared_ptr<prx::lqr_t> l, Types... args)
{
  l->set_goal(args...);
}

// BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(lqr_compute_controls_overloads, prx::lqr_t::compute_controls, 0, 1)
void (prx::lqr_t::*lqr_compute_controls_0)() = &prx::lqr_t::compute_controls;
void (prx::lqr_t::*lqr_compute_controls_1)(prx::space_point_t&) = &prx::controller_t::compute_controls;

void bindings()
{
  class_<prx::lqr_t, std::shared_ptr<prx::lqr_t>, bases<prx::controller_t>>("lqr", no_init)
      // class_<prx::lqr_t, bases<controller_wrap>>("lqr", no_init)
      .def("__init__",
           make_constructor(&init_as_ptr<prx::lqr_t, prx::system_ptr_t, std::string>, default_call_policies()))
      .def("__init__",
           make_constructor(&init_as_ptr<prx::lqr_t, prx::system_ptr_t, Eigen::MatrixXd, Eigen::MatrixXd, std::string>,
                            default_call_policies()))
      // .def(init<std::shared_ptr<prx::lti_t>, std::string>())
      // .def(init<std::shared_ptr<prx::lti_t>, Eigen::MatrixXd, Eigen::MatrixXd, std::string>())
      .def("set_Q", &prx::lqr_t::set_Q)
      .def("set_R", &prx::lqr_t::set_R)
      // .def("compute_controls", &prx::lqr_t::compute_controls, lqr_compute_controls_overloads())
      .def("compute_controls", lqr_compute_controls_0)
      .def("compute_controls", lqr_compute_controls_1)
      .def("compute_K", &prx::lqr_t::compute_K)
      .def("get_K", &prx::lqr_t::get_K)
      .def("set_goal", &set_goal<Eigen::VectorXd>)
      .def("set_goal", &set_goal<Eigen::VectorXd, Eigen::VectorXd>)
      .def("set_goal", &set_goal<prx::space_point_t>)
      .def("set_goal", &set_goal<prx::space_point_t, prx::space_point_t>)
      // .def("", &prx::lqr_t::)
      // .def("", &prx::lqr_t::)
      ;
}
}  // namespace lqr
}  // namespace controllers
}  // namespace simulation
}  // namespace pyprx