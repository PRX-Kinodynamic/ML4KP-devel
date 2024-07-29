#include "prx/simulation/controllers/noisy_controller.hpp"

namespace pyprx
{
namespace simulation
{
namespace controllers
{
namespace noisy_controller
{

template <class T>
void (prx::noisy_controller_t<T>::*noisy_controller_compute_controls_0)() =
    &prx::noisy_controller_t<T>::compute_controls;
template <class T>
void (prx::noisy_controller_t<T>::*noisy_controller_compute_controls_1)(prx::space_point_t&) =
    &prx::noisy_controller_t<T>::compute_controls;

template <class T, typename... Types>
void bind_noisy_controller(const std::string& name)
{
  class_<prx::noisy_controller_t<T>, std::shared_ptr<prx::noisy_controller_t<T>>, bases<prx::controller_t>,
         boost::noncopyable>(name.c_str(), no_init)
      // class_<T, std::shared_ptr<T>, bases<prx::controller_t>, boost::noncopyable>(name, init<T, prx::controller_t,
      // Types...>()) class_<prx::noisy_controller_t<T>, bases<prx::controller_t>>(name, no_init)
      .def("__init__", make_constructor(&init_as_ptr<prx::noisy_controller_t<T>, prx::controller_ptr_t, Types...>,
                                        default_call_policies()))
      .def("compute_controls", noisy_controller_compute_controls_0<T>)
      .def("compute_controls", noisy_controller_compute_controls_1<T>);
}

void bindings()
{
  bind_noisy_controller<std::normal_distribution<double>, double, double>("noisy_gaussian_controller");
  bind_noisy_controller<std::uniform_real_distribution<double>, double, double>("noisy_uniform_controller");
}
}  // namespace noisy_controller
}  // namespace controllers
}  // namespace simulation
}  // namespace pyprx