#pragma once
namespace prx
{
namespace fg
{

namespace mushrTypes
{
// using KeyFunction = std::function<gtsam::Key(const std::size_t&)>;
// using KeyFunctionStr = std::function<gtsam::Key(const std::string&)>;
using SF = symbol_factory_t;
// clang-format off
inline gtsam::Key k_X(const std::size_t& ti, const std::size_t traj=0) { return SF::create_hashed_symbol("x^{",traj,"}_{", ti, "}"); };
inline gtsam::Key k_U(const std::size_t& ti, const std::size_t traj=0) { return SF::create_hashed_symbol("u^{",traj,"}_{", ti, "}"); };
inline gtsam::Key k_Xd(const std::size_t& ti, const std::size_t traj=0) { return SF::create_hashed_symbol("xd^{",traj,"}_{", ti, "}"); };
inline gtsam::Key k_Ub(const std::size_t& ti, const std::size_t traj=0) { return SF::create_hashed_symbol("Ub^{",traj,"}_{", ti, "}"); };
inline gtsam::Key k_ZX(const std::size_t& ti) { return SF::create_hashed_symbol("Z^x_{", ti, "}"); };
inline gtsam::Key k_Ps(const std::string& s) { return SF::create_hashed_symbol("P_{", s, "}"); };
// clang-format on

const Eigen::Index UbarDim = 3;
using State = Eigen::Vector<double, 3>;
using StateDot = Eigen::Vector<double, 3>;
using Control = Eigen::Vector<double, 2>;
using Ubar = Eigen::Vector<double, UbarDim>;
using Parameters = Eigen::Vector<double, 6>;
using ParamsUbarU = Eigen::Vector<double, 4>;

static inline double positive_slope(const ParamsUbarU& param)
{
  return param[0];
}
static inline double steering_offset(const ParamsUbarU& params)
{
  return params[1];
}
static inline double velocity_gain(const ParamsUbarU& params)
{
  return params[2];
}
static inline double desired_velocity_gain(const ParamsUbarU& params)
{
  return params[3];
}
static inline double bound(const double value, const double min_bound, const double max_bound)
{
  return std::max(std::min(value, max_bound), min_bound);
}
static inline double& vel_delta_max(Parameters& params)
{
  return params[0];
}
static inline double& vel_delta_min(Parameters& params)
{
  return params[1];
}
static inline double& steering_gain(Parameters& params)
{
  return params[2];
}
static inline double& steering_offset(Parameters& params)
{
  return params[3];
}
static inline double& velocity_min(Parameters& params)
{
  return params[4];
}
static inline double& velocity_max(Parameters& params)
{
  return params[5];
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

static inline double velocity_min(const Parameters& params)
{
  return params[4];
}
static inline double velocity_max(const Parameters& params)
{
  return params[5];
}
static inline double steering(const Control& u)
{
  return u[0];
}
static inline double steering(const Control& u, const Parameters& params)
{
  const double us{ u[0] * steering_gain(params) };
  // return bound(us, -1.0, 1.0);
  return us;
}
static inline double desired_velocity(const Control& u)
{
  return u[1];
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
}  // namespace mushrTypes
}  // namespace fg
}  // namespace prx