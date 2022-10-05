#include <iostream>
#include <boost/python.hpp>

#include "prx/simulation/plants/lander_LD.hpp"
 
using namespace boost::python;

void  (prx::lander_meditch_ctrl_t::*lander_meditch_ctrl_compute_controls_0)() = &prx::lander_meditch_ctrl_t::compute_controls;

void pyprx_simulation_plants_lander_LD()
{
    class_<prx::lander_LD_t, std::shared_ptr<prx::lander_LD_t>, bases<prx::plant_t>>("lander_LD", no_init)
        .def("__init__", make_constructor(&create_system_ptr<prx::lander_LD_t>, default_call_policies(), (arg("path"))))
        ;
    class_<prx::lander_meditch_ctrl_t, std::shared_ptr<prx::lander_meditch_ctrl_t>, bases<prx::controller_t>>("lander_meditch_ctrl", no_init)
        .def("__init__", make_constructor(&init_as_ptr<prx::lander_meditch_ctrl_t, prx::system_ptr_t, std::string>, default_call_policies(), (arg("plant"), arg("name"))))
        .def("compute_controls", lander_meditch_ctrl_compute_controls_0)
        ;
}
