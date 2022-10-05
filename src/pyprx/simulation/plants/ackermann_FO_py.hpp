#include <iostream>
#include <boost/python.hpp>

#include "prx/simulation/plants/ackermann_FO.hpp"
 
using namespace boost::python;


void pyprx_simulation_plants_ackermann_FO()
{
    class_<prx::ackermann_FO, std::shared_ptr<prx::ackermann_FO>, bases<prx::plant_t>>("ackermann_FO", no_init)
        .def("__init__", make_constructor(&create_system_ptr<prx::ackermann_FO>, default_call_policies(), (arg("path"))))
        ;
}
