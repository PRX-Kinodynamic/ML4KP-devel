#include <iostream>
#include <boost/python.hpp>
#include "prx/simulation/plants/types/linear_time_variant.hpp"

using namespace boost::python;

// struct lti_wrap : prx::lti_t, wrapper<prx::lti_t>
// {
// 	public:
//     lti_wrap(const lti_t& other) : lti_t(other){};

//     lti_wrap(const std::string& path) : prx::lti_t(path){};

//     bool linearize_0()
//     {
//         return this->get_override("linearize")();
//     }

//     bool check()
//     {
//         return this->get_override("check")();
//     }

//     void update_configuration()
//     {
//         this->get_override("update_configuration")();
//     }

//     void compute_derivative()
//     {
//         this->get_override("compute_derivative")();
//         // this -> compute_derivative();
//     }

// };
bool linearize_0(std::shared_ptr<prx::ltv_t> _ltv)
{
  return _ltv->linearize();
  // return std::static_pointer_cast<prx::lti_t>(ltv) -> linearize();
}

bool linearize_2(std::shared_ptr<prx::ltv_t> _ltv, prx::space_point_t xt, prx::space_point_t ut)
{
  return _ltv->linearize(xt, ut);
}

bool linearize_3(std::shared_ptr<prx::ltv_t> _ltv, prx::space_point_t xt, prx::space_point_t ut, double epsilon)
{
  return _ltv->linearize(xt, ut, epsilon);
}

void pyprx_simulation_plants_types_ltv()
{
  // class_<ltv_wrap, std::shared_ptr<ltv_wrap>, bases<prx::lti_t>, boost::noncopyable>("ltv", no_init)
  class_<prx::ltv_t, std::shared_ptr<prx::ltv_t>, bases<prx::lti_t>, boost::noncopyable>("ltv", no_init)
      .def("__init__",
           make_constructor(&init_as_ptr<prx::lti_t, prx::system_ptr_t>, default_call_policies(), (arg("_sys_ptr"))))
      // .def("__init__", make_constructor(&init_as_ptr<prx::lti_t, const lti_t&>, default_call_policies(),
      // (arg("other"))))
      .def("linearize", &linearize_0)
      .def("linearize", &linearize_2)
      .def("linearize", &linearize_3)
      // TODO: Add "std::vector<double*>" class
      // .def("derivative_memory", &get_plant_t_derivative_memory<std::vector<double*>>,
      // &set_plant_t_derivative_memory<std::vector<double*>>)
      ;

  // iterable_converter()
  //     .from_python<std::vector<prx::system_ptr_t> >()
  //     ;
  // pyprx_simulation_plants_acrobot();
}
