#include "prx/simulation/controllers/ackermann_FO_ctrl.hpp"

void pyprx_simulation_controllers_ackermann_FO_ctrl()
{
    class_<prx::ackermann_FO_ctrl_t, bases<prx::controller_t>>("ackermann_FO_ctrl", no_init)
        .def(init<prx::ackermann_FO_ctrl_t>())
		.def(init<prx::system_ptr_t, std::string>())
		.def("compute_controls", &prx::ackermann_FO_ctrl_t::compute_controls)
		.def("set_gains", &prx::ackermann_FO_ctrl_t::set_gains)
		// .def("", &prx::ackermann_FO_ctrl_t::)
		// .def("", &prx::ackermann_FO_ctrl_t::)
        ;
}