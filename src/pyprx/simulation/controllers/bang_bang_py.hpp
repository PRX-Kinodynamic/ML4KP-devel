#include "prx/simulation/controllers/bang_bang.hpp"


void pyprx_simulation_controllers_bang_bang()
{
    class_<prx::bang_bang_t, std::shared_ptr<prx::bang_bang_t>, bases<prx::controller_t>>("bang_bang", no_init)
		.def("__init__", make_constructor(&init_as_ptr<prx::bang_bang_t, 
						  prx::system_ptr_t, std::vector<std::vector<double>>, std::string >, default_call_policies()))
		.def("compute_controls", &prx::bang_bang_t::compute_controls)
		.def("set_control", &prx::bang_bang_t::set_control)
		.def("get_control_at", &prx::bang_bang_t::get_control_at)
		.def("get_num_ctrls", &prx::bang_bang_t::get_num_ctrls)
	;
		
}