#include <iostream>
#include <boost/python.hpp>
#include "prx/simulation/plants/types/noisy_plant.hpp"

using namespace boost::python;

template <class T, class... Types>
void bind_noisy_plant(const std::string& name)
{
  class_<prx::noisy_plant_t<T>, std::shared_ptr<prx::noisy_plant_t<T>>, bases<prx::plant_t>, boost::noncopyable>(
      name.c_str(), no_init)
      .def("__init__",
           make_constructor(&init_as_ptr<prx::noisy_plant_t<T>, prx::system_ptr_t, Types...>, default_call_policies()))
      // .def("get_state_space", &prx::noisy_plant_t<T>::get_state_space, return_internal_reference<>())
      ;
}

void pyprx_simulation_plants_types_noisy_plant()
{
  bind_noisy_plant<prx::uniform_noise_t, double, double>("uniform_noisy_plant");
}