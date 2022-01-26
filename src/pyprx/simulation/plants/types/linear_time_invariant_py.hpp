#include <iostream>
#include <boost/python.hpp>
#include "prx/simulation/plants/types/linear_time_invariant.hpp"
 
using namespace boost::python;

struct lti_wrap : prx::lti_t, wrapper<prx::lti_t>
{
	public:
    lti_wrap(const lti_t& other) : lti_t(other){};

    lti_wrap(const std::string& path) : prx::lti_t(path){};

    // bool linearize_0()
    // {
    //     return this->get_override("linearize")();
    // }

    bool check() 
    {
        return this->get_override("check")();
    }

    void compute_derivative() 
    {
        this->get_override("compute_derivative")();
        // this -> compute_derivative();
    }

};

void pyprx_simulation_plants_types_lti()
{
   	class_<prx::lti_t, std::shared_ptr<prx::lti_t>, bases<prx::plant_t>, boost::noncopyable>("lti", no_init)
      // .def("__init__", make_constructor(&init_as_ptr<prx::lti_t, std::string>, default_call_policies(), (arg("path")) ))
      // .def("__init__", make_constructor(&init_as_ptr<prx::lti_t, const lti_t&>, default_call_policies(), (arg("other"))))
      .def("linearize", &prx::lti_t::linearize)
      .def("check", &prx::lti_t::check)
      .def("linear_derivative", &prx::lti_t::linear_derivative)
      .def("linear_derivative_and_output", &prx::lti_t::linear_derivative_and_output)
      .def("discretize", &prx::lti_t::discretize)
      .def("get_state_space", &prx::lti_t::get_state_space, return_internal_reference<>())
      .def("get_control_space", &prx::lti_t::get_control_space, return_internal_reference<>())
      // .def("update_configuration", pure_virtual(&lti_wrap::update_configuration))
      .def("compute_derivative", pure_virtual(&lti_wrap::compute_derivative))
      .def("get_A", &prx::lti_t::get_A)
      .def("get_B", &prx::lti_t::get_B)
      .def("get_C", &prx::lti_t::get_C)
      .def("get_D", &prx::lti_t::get_D)
      // TODO: Add "std::vector<double*>" class
      // .def("derivative_memory", &get_plant_t_derivative_memory<std::vector<double*>>, &set_plant_t_derivative_memory<std::vector<double*>>)
   		;
        
    // iterable_converter()
    //     .from_python<std::vector<prx::system_ptr_t> >()
   	//     ;
	// pyprx_simulation_plants_acrobot();
   	
}
