#pragma once

#include "prx/simulation/controller.hpp"
#include "prx/utilities/general/noise.hpp"
// #include "prx/simulation/plants/types/linear_time_invariant.hpp"

namespace prx
{
template <class T>
class noisy_controller_t;

typedef noisy_controller_t<gaussian_noise_t> noisy_gaussian_controller_t;
typedef noisy_controller_t<uniform_noise_t> noisy_uniform_controller_t;

template <class T>
class noisy_controller_t : public controller_t
{
public:
  noisy_controller_t(system_ptr_t _plant, std::string _name) = delete;

  template <class... Types>
  noisy_controller_t(controller_ptr_t _ctrl, Types... args) : controller_t(_ctrl), noise(args...)
  {
    ctrl = _ctrl;
    // plant = ctrl -> plant;
    u = get_control_space()->make_point();
    // noise = _noise;
  }

  using controller_t::compute_controls;
  virtual void compute_controls() override
  {
    ctrl->compute_controls();
    // This is not the most efficient way...
    get_control_space()->copy_to_point(u);
    noise.add_noise(u);
    get_control_space()->copy_from_point(u);
    get_control_space()->enforce_bounds();
  }

private:
  controller_ptr_t ctrl;
  space_point_t u;
  noise_t<T> noise;
};
}  // namespace prx