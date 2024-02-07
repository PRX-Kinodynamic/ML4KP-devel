#pragma once
#include <array>
#include <numeric>
#include <gtsam/config.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "prx/utilities/math/first_order_derivative.hpp"
#include "prx/factor_graphs/factors/noise_model_factor.hpp"
#include <prx/factor_graphs/utilities/symbols_factory.hpp>
// #include "prx/factor_graphs/factors/noise_model_factor.hpp"
// #include "prx/factor_graphs/utilities/perception/camera.hpp"
// #include "prx/factor_graphs/utilities/symbols_factory.hpp"

namespace prx
{
namespace fg
{
class mushr_factor_t : public noise_model_1p3_factor_t<4, 4, 2, 5>
{
  using Base = noise_model_1p2_factor_t<2, 3, 12>;

public:
  using Observation = Base::X0;
  using State = Base::X1;
  using Control = Base::X2;
  using Parameters = Base::X3;

  // Observation at t
  // (State, control) at t-1
  // x_t = f(x_{t-1}, u_{t-1})
  mushr_factor_t(gtsam::Key observation, gtsam::Key State, gtsam::Key control, gtsam::Key parameters,
                 const gtsam::noiseModel::Base::shared_ptr& cost_model, double length = 0.2965)
    : Base(key_world_position, key_projection, cost_model), _pixel(pixel), _length(length)
  {
    _g << 0, 1, 0, 0, 0, 0;
  }

  virtual Pixel compute_error(const State& state, const Control& u, const Parameters& params) const override
  {
    _g(1, 0) = std::cos(theta(state));
    _g(2, 0) = std::sin(theta(state));
    const double vel_delta{ desired_velocity(u) - current_velocity(state) };
    const double vel_delta_cap{ std::max(std::min(_vel_delta, _vel_delta_max), -_vel_delta_max) };
    const double w{ current_velocity(state) * std::tan(steering(u)) / _length };
    _qdot = _g * Eigen::Vector2d(_current_vel, w);
  }

  static inline double steering(Control& u)
  {
    return u[0];
  }
  static inline double desired_velocity(Control& u)
  {
    return u[1];
  }
  static inline double x(State& state)
  {
    return state[0];
  }
  static inline double y(State& state)
  {
    return state[1];
  }
  static inline double theta(State& state)
  {
    return state[2];
  }
  static inline double current_velocity(State& state)
  {
    return state[3];
  }

private:
  Eigen::Matrix<double, 3, 2> _g;
}
}  // namespace prx
