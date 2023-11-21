#include <iostream>
#include <boost/python.hpp>

#include "prx/simulation/plants/mountain_car.hpp"

using namespace boost::python;

namespace pyprx
{
namespace simulation
{
namespace plants
{
namespace mountain_car
{

void bindings()
{
  class_<prx::mountain_car_t, std::shared_ptr<prx::mountain_car_t>, bases<prx::plant_t>>("mountain_car", no_init)
      .def("__init__",
           make_constructor(&create_system_ptr<prx::mountain_car_t>, default_call_policies(), (arg("path"))));
}

}  // namespace mountain_car
}  // namespace plants
}  // namespace simulation
}  // namespace pyprx