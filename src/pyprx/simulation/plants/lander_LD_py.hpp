#include <iostream>
#include <boost/python.hpp>

#include "prx/simulation/plants/lander_LD.hpp"
 
using namespace boost::python;


void pyprx_simulation_plants_lander_LD()
{
    class_<prx::lander_LD_t, std::shared_ptr<prx::lander_LD_t>, bases<prx::plant_t>>("lander_LD", no_init)
        .def("__init__", make_constructor(&create_system_ptr<prx::lander_LD_t>, default_call_policies(), (arg("path"))))
        ;
}
