#include <iostream>
#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>
#include "prx/planning/noisy_world_model.hpp"
#include "prx/simulation/system_group.hpp"

using namespace boost::python;

template <class T, class... Types>
void bind_noisy_world_model(const std::string& name)
{
  class_<prx::noisy_world_model_t<T>, std::shared_ptr<prx::noisy_world_model_t<T>>, bases<prx::world_model_t>,
         boost::noncopyable>(name.c_str(), no_init)
      .def("__init__",
           make_constructor(&init_as_ptr<prx::noisy_world_model_t<T>, const std::vector<prx::system_ptr_t>&,
                                         const std::vector<std::shared_ptr<prx::movable_object_t>>&, Types...>,
                            default_call_policies()));
}

void pyprx_planning_noisy_world_model_py()
{
  bind_noisy_world_model<prx::uniform_noise_t, double, double>("uniform_noisy_world_model");
}
