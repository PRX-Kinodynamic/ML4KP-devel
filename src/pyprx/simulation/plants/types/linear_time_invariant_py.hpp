#include <iostream>
#include <boost/python.hpp>
#include "prx/simulation/plants/types/linear_time_invariant.hpp"
 
using namespace boost::python;

struct lti_wrap : prx::lti_t, wrapper<prx::lti_t>
{
	public:
    lti_wrap(const lti_t& other) : lti_t(other){};

    lti_wrap(const std::string& path) : prx::lti_t(path){};

    bool linearize_0()
    {
        return this->get_override("linearize")();
    }

    bool check() 
    {
        return this->get_override("check")();
    }

    void update_configuration()
    {
        this->get_override("update_configuration")();
    }

    void compute_derivative() 
    {
        this->get_override("compute_derivative")();
        // this -> compute_derivative();
    }

};

void pyprx_simulation_plants_types_lti()
{
   	class_<lti_wrap, std::shared_ptr<lti_wrap>, bases<prx::plant_t>>("lti", no_init)
      // .def("__init__", make_constructor(&init_as_ptr<prx::plant_t, std::string>, default_call_policies()))
      .def("linearize", &prx::lti_t::linearize)
      .def("check", &prx::lti_t::check)
      .def("derivative", &prx::lti_t::derivative)
      .def("derivative_and_output", &prx::lti_t::derivative_and_output)
      .def("discretize", &prx::lti_t::discretize)
      .def("get_state_space", &prx::lti_t::get_state_space, return_internal_reference<>())
      .def("get_control_space", &prx::lti_t::get_control_space, return_internal_reference<>())
      .def("update_configuration", &lti_wrap::update_configuration)
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
