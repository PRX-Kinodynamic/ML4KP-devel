#include <iostream>
#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>
#include "prx/planning/condition_check.hpp"
#include "prx/planning/planner_functions/planner_functions.hpp"

using namespace boost::python;

void pyprx_planning_condition_check_py()
{
        def("create_default_goal_check", &prx::create_default_goal_check);
        class_<prx::custom_check_t>("custom_check")
                .def("__call__", &prx::custom_check_t::operator() )
                .def("wrap", &create_function<prx::custom_check_t, bool>).staticmethod("wrap")
                ;

   	class_<prx::condition_check_t, prx::condition_check_t*>("condition_check", no_init )//init<std::string, double>())   
                .def("__init__", make_constructor(&init_as_ptr<prx::condition_check_t, std::string, double>, default_call_policies(), (args("type"), args("check"))) )
   		.def("__init__", make_constructor(&init_as_ptr<prx::condition_check_t, prx::custom_check_t>, default_call_policies(), (args("custom_check"))) )
                .def("reset", &prx::condition_check_t::reset)
                .def("check", &prx::condition_check_t::check)
                .def("time", &prx::condition_check_t::time)
                .def("iterations", &prx::condition_check_t::iterations)
                .def("get_check_value", &prx::condition_check_t::get_check_value)
                .def("set_check_value", &prx::condition_check_t::set_check_value)
                .def("add_condition", &prx::condition_check_t::add_condition)
                .def("get_available_types", &prx::condition_check_t::get_available_types)
                .def("print_available_types", &prx::condition_check_t::print_available_types)
        // .def("condition", &prx::condition_check_t::condition)
   	    ;
   	
}