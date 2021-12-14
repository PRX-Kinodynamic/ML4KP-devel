#include "prx/simulation/controllers/lqr.hpp"

void set_eigen_goal(prx::lqr_t* l, Eigen::VectorXd _goal)
{
	l -> set_goal(_goal);
}

void set_prx_goal(prx::lqr_t* l, prx::space_point_t _goal)
{
	l -> set_goal(_goal);
}


void pyprx_simulation_controllers_lqr()
{
    class_<prx::lqr_t, bases<prx::controller_t>>("lqr", no_init)
        .def(init<std::shared_ptr<prx::lti_t>, std::string>())
		.def(init<std::shared_ptr<prx::lti_t>, Eigen::MatrixXd, Eigen::MatrixXd, std::string>())
		.def("set_Q", &prx::lqr_t::set_Q)
		.def("set_R", &prx::lqr_t::set_R)
		.def("compute_controls", &prx::lqr_t::compute_controls)
		.def("compute_K", &prx::lqr_t::compute_K)
		.def("get_K", &prx::lqr_t::get_K)
		.def("set_goal", &set_eigen_goal)
		.def("set_goal", &set_prx_goal)
		// .def("", &prx::lqr_t::)
		// .def("", &prx::lqr_t::)
        ;
}