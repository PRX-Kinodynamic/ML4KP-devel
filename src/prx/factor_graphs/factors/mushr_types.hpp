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

namespace Parameters
{
constexpr double L{ 0.2965 };

}
namespace State
{
using type = Eigen::Vector<double, 3>;
constexpr std::size_t x{ 0 };
constexpr std::size_t y{ 1 };
constexpr std::size_t theta{ 2 };
}  // namespace State

namespace StateDot
{
using type = Eigen::Vector<double, 3>;
constexpr std::size_t xdot{ 0 };
constexpr std::size_t ydot{ 1 };
constexpr std::size_t thetadot{ 2 };
}  // namespace StateDot

namespace Ubar
{
constexpr std::size_t Dim{ 2 };
constexpr std::size_t ParamsDim{ 3 };
using type = Eigen::Vector<double, Dim>;
using params = Eigen::Vector<double, ParamsDim>;

constexpr std::size_t velocity{ 0 };
constexpr std::size_t beta{ 1 };

constexpr std::size_t accel_slope{ 0 };
constexpr std::size_t steering_param{ 1 };
constexpr std::size_t max_vel_param{ 2 };

}  // namespace Ubar

namespace Control
{
using type = Eigen::Vector<double, 2>;
constexpr std::size_t vel_desired{ 0 };
constexpr std::size_t steering{ 1 };
}  // namespace Control

}  // namespace mushrTypes
}  // namespace fg
}  // namespace prx