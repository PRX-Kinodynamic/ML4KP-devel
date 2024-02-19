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

namespace mushr
{
using Control = Eigen::Vector<double, 2>;
using Parameters = Eigen::Vector<double, 6>;

static inline double bound(const double value, const double min_bound, const double max_bound)
{
  return std::max(std::min(value, max_bound), min_bound);
}
static inline double vel_delta_max(const Parameters& params)
{
  return params[0];
}
static inline double vel_delta_min(const Parameters& params)
{
  return params[1];
}
static inline double steering_gain(const Parameters& params)
{
  return params[2];
}
static inline double steering_offset(const Parameters& params)
{
  return params[3];
}
static inline double velocity_min(const Parameters& params)
{
  return params[4];
}
static inline double velocity_max(const Parameters& params)
{
  return params[5];
}
static inline double steering(const Control& u, const Parameters& params)
{
  const double us{ u[0] * steering_gain(params) + steering_offset(params) };
  // return bound(us, -1.0, 1.0);
  return us;
}
static inline double desired_velocity(const Control& u, const Parameters& params)
{
  return bound(u[1], velocity_min(params), velocity_max(params));
}
template <typename State>
static inline double x(const State& state)
{
  return state[0];
}
template <typename State>
static inline double y(const State& state)
{
  return state[1];
}
template <typename State>
static inline double theta(const State& state)
{
  return state[2];
}
template <typename State>
static inline double current_velocity(const State& state)
{
  return state[3];
}
}  // namespace mushr

class mushr_factor_t : public noise_model_4factor_t<4, 4, 2, 6>
{
  using Base = noise_model_4factor_t<4, 4, 2, 6>;

public:
  using State = Base::X0;
  using Control = Base::X2;
  using Parameters = Base::X3;

  // Observation at t
  // (State, control) at t-1
  // x_t = f(x_{t-1}, u_{t-1})
  mushr_factor_t(gtsam::Key state_i, gtsam::Key state_j, gtsam::Key control, gtsam::Key parameters, double dt,
                 const gtsam::noiseModel::Base::shared_ptr& cost_model, double length = 0.2965)
    : Base(state_i, state_j, control, parameters, cost_model), _length(length), _dt(dt)
  {
    _g << 0, 1, 0, 0, 0, 0;
  }

  virtual State compute_error(const State& state_i, const State& state_j, const Control& u,
                              const Parameters& params) const override
  {
    const State xdot{ compute_xdot(state_i, u, params) };
    const State prediction{ state_i + xdot * _dt };

    return prediction - state_j;
  }

  State compute_xdot(const State& state, const Control& u, const Parameters& params) const
  {
    _g(1, 0) = std::cos(mushr::theta(state));
    _g(2, 0) = std::sin(mushr::theta(state));
    const double vel_delta{ mushr::desired_velocity(u, params) - mushr::current_velocity(state) };
    // const double vel_delta_cap{ std::max(std::min(vel_delta, vel_delta_max(params)), vel_delta_min(params)) };
    const double vel_delta_cap{ mushr::bound(vel_delta, mushr::vel_delta_min(params), mushr::vel_delta_max(params)) };
    const double w{ mushr::current_velocity(state) * std::tan(mushr::steering(u, params)) / _length };
    return (State() << _g * Eigen::Vector2d(mushr::current_velocity(state), w), vel_delta).finished();
  }

  void eval_to_stream(gtsam::Values& values, std::ostream& os)
  {
    const State xi{ values.at<State>(key<1>()) };
    const State xj{ values.at<State>(key<2>()) };
    const Control ui{ values.at<Control>(key<3>()) };
    const Parameters p{ values.at<Parameters>(key<4>()) };

    // const double
    os << _dt << " ";             // 1,
    os << xi.transpose() << " ";  // 2, 3, 4, 5
    os << xj.transpose() << " ";  // 6, 7, 8, 9
    os << ui.transpose() << " ";  // 10, 11,
    os << p.transpose() << "\n";  // 12, 13, 14, 15, 16, 17
  }

private:
  mutable Eigen::Matrix<double, 3, 2> _g;
  const double _dt;
  const double _length;
};

class mushr_control_to_vel_t : public noise_model_3factor_t<2, 3, 3>
{
  using Base = noise_model_3factor_t<2, 3, 3>;

public:
  using Control = Base::X0;
  using Velocity = Base::X1;
  using State = Base::X2;

  // Observation at t
  // (State, control) at t-1
  // x_t = f(x_{t-1}, u_{t-1})
  mushr_control_to_vel_t(gtsam::Key ui, gtsam::Key vi, gtsam::Key xi, const double length,
                         const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(ui, vi, xi, cost_model), _length(length)
  {
  }

  virtual Control compute_error(const Control& ui, const Velocity& vi, const State& xi) const override
  {
    return Control{};
  }

  Velocity predict_velocity()
  {
    // const double w{ current_velocity(state) * std::tan(steering(u, params)) / _length };
    // const Control ctrl{ Control(current_velocity(state), w) };

    // return _g * ctrl;
  }

  State compute_xdot(const State& state, const Control& u) const
  {
    // _g(1, 0) = std::cos(theta(state));
    // _g(2, 0) = std::sin(theta(state));
    // const double vel_delta{ desired_velocity(u, params) - current_velocity(state) };
    // // const double vel_delta_cap{ std::max(std::min(vel_delta, vel_delta_max(params)), vel_delta_min(params)) };
    // const double vel_delta_cap{ bound(vel_delta, vel_delta_min(params), vel_delta_max(params)) };
    // const double w{ current_velocity(state) * std::tan(steering(u, params)) / _length };
    // return (State() << _g * Eigen::Vector2d(current_velocity(state), w), vel_delta).finished();
  }

  void eval_to_stream(gtsam::Values& values, std::ostream& os)
  {
    // const State xi{ values.at<State>(key<1>()) };
    // const State xj{ values.at<State>(key<2>()) };
    // const Control ui{ values.at<Control>(key<3>()) };
    // const Parameters p{ values.at<Parameters>(key<4>()) };

    // // const double
    // os << _dt << " ";             // 1,
    // os << xi.transpose() << " ";  // 2, 3, 4, 5
    // os << xj.transpose() << " ";  // 6, 7, 8, 9
    // os << ui.transpose() << " ";  // 10, 11,
    // os << p.transpose() << "\n";  // 12, 13, 14, 15, 16, 17
  }

private:
  mutable Eigen::Matrix<double, 3, 2> _g;
  const double _length;
};
}  // namespace fg
}  // namespace prx
