#include <iostream>
#include <boost/python.hpp>
#include "prx/simulation/controller.hpp"

struct controller_wrap : prx::controller_t, wrapper<prx::controller_t>
{
	controller_wrap(const prx::controller_t& other) : controller_t(other){};
	controller_wrap(prx::system_ptr_t _plant, std::string _name) : prx::controller_t(_plant, _name){};
	controller_wrap(prx::system_ptr_t _plant) : prx::controller_t(_plant){};

    void compute_controls()
    {
        this->get_override("compute_controls")();
    }

	void propagate_1(const double simulation_step)
	{
        this->get_override("propagate")(simulation_step);
	}

	void propagate_2(const double simulation_step, const prx::propagate_step step)
	{
        this->get_override("propagate")(simulation_step, step);
	}



};

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(controller_propagate_overloads, propagate, 1, 2)

void pyprx_simulation_controller()
{

	class_<controller_wrap, boost::noncopyable>("controller", init<prx::system_ptr_t>())
		.def(init<prx::system_ptr_t, std::string>())
        // .def("__init__", make_constructor(&init_as_ptr<system_wrap, std::string>, default_call_policies()))
		// .def("__init__", make_constructor(&system_wrap, default_call_policies(), (arg("path")) ))
        // .def("__init__", make_constructor(&fromAxisAngle, default_call_policies(),(arg("axis"),  arg("angle"))))
		.def("get_state_space", &prx::controller_t::get_state_space, return_internal_reference<>())
		.def("get_control_space", &prx::controller_t::get_control_space, return_internal_reference<>())
		.def("compute_controls", &controller_wrap::compute_controls)
	 	.def("propagate", &controller_wrap::propagate_1)
	 	.def("propagate", &controller_wrap::propagate_2)
		// .def("propagate", &controller_wrap::propagate, controller_propagate_overloads())
	 	.def("set_plan", &prx::controller_t::set_plan)
	 	.def("get_plan", &prx::controller_t::get_plan)
	 	.def("init_plan", &prx::controller_t::init_plan)

	 	// .def("compute_control", pure_virtual(&prx::system_t::compute_control))
	 	// .def("compute_stopping_maneuver", &prx::system_t::compute_stopping_maneuver)
	 	// .def("finalize_system_tree", &prx::system_t::finalize_system_tree)
	 	// .def("set_state_space_bounds", pure_virtual(&prx::system_t::set_state_space_bounds))
	 	// .def("get_pathname", &prx::system_t::get_pathname)
	 	// .def("get_system_type", &prx::system_t::get_system_type)
   //    	.def("get_state_space", &system_wrap::get_system_wrap_state_space, return_internal_reference<>())
   //    	.def("set_state_space", &system_wrap::set_system_wrap_state_space, return_internal_reference<>())
   //    	.def("get_input_control_space", &system_wrap::get_system_wrap_input_control_space, return_internal_reference<>())
   //    	.def("set_input_control_space", &system_wrap::set_system_wrap_input_control_space, return_internal_reference<>())
	 	// .def("get_parameter_space", &system_wrap::get_system_wrap_parameter_space, return_internal_reference<>())
   //    	.def("set_parameter_space", &system_wrap::set_system_wrap_parameter_space, return_internal_reference<>())
	 	// .def("", &prx::system_t::)
	 	// .def("", &prx::system_t::)
	 	// .def("", &prx::system_t::)
		// virtual inline const space_t* get_state_space() const
		// .def("create_ptr", &create_ptr<prx::system_t, prx::system_t> ).staticmethod("create_ptr")
		;
	// register_ptr_to_python< prx::system_ptr_t >();

	// class_<std::vector<prx::system_ptr_t>>("vector_system")
 //   		.def(vector_indexing_suite<std::vector<prx::system_ptr_t>>())
	// 	;

}