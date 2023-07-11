#include <iostream>
#include <boost/python.hpp>
#include <boost/python/copy_const_reference.hpp>
#include <boost/python/return_value_policy.hpp>

#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/spaces/noisy_space.hpp"

template <class T, class S>
void copy_to_vector_wrapper(T& s, S& pt)
{
  s.copy_to_vector(pt);
}

template <class T, class... Types>
void bind_noisy_space(const std::string& name)
{
  class_<prx::noisy_space_t<T>, std::shared_ptr<prx::noisy_space_t<T>>, bases<prx::space_t>>(name.c_str(), no_init)
      .def("__init__",
           make_constructor(&init_as_ptr<prx::noisy_space_t<T>, prx::space_t*, Types...>, default_call_policies()))
      // .def("add_noise", add_noise_space_point_wrapper)
      .def("copy_to_point", &prx::noisy_space_t<T>::copy_to_point)
      .def("copy_to_vector", copy_to_vector_wrapper<prx::noisy_space_t<T>, std::vector<double>>)
      .def("copy_to_vector", copy_to_vector_wrapper<prx::noisy_space_t<T>, Eigen::VectorXd>);
}

void pyprx_utilities_spaces_noisy_space()
{
  bind_noisy_space<prx::uniform_noise_t, double, double>("uniform_noisy_space");
}