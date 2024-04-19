#pragma once
#include <array>
#include <numeric>

#include "prx/simulation/plant.hpp"

namespace prx
{
inline double angle_diff(const double a, const double b)
{
  return std::atan2(std::sin(a - b), std::cos(a - b));
}

namespace mushrTypes
{

// using Parameters = Eigen::Vector<double, 6>;
using ParamsUbarU = Eigen::Vector<double, 3>;

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
using type = Eigen::Vector<double, 2>;
using params = Eigen::Vector<double, 3>;
constexpr std::size_t velocity{ 0 };
constexpr std::size_t beta{ 1 };

constexpr std::size_t accel_slope{ 0 };

}  // namespace Ubar

namespace Control
{
using type = Eigen::Vector<double, 2>;
constexpr std::size_t vel_desired{ 0 };
constexpr std::size_t steering{ 1 };
}  // namespace Control

}  // namespace mushrTypes
class mushrFG_t : public prx::plant_t
{
public:
  mushrFG_t(const std::string& path);
  ~mushrFG_t();
  virtual void propagate(const double simulation_step) override final;
  virtual void update_configuration() override;
  virtual void compute_derivative() override final;

protected:
  mushrTypes::State::type _state;
  mushrTypes::StateDot::type _state_dot;
  mushrTypes::Control::type _ctrl;
  mushrTypes::Ubar::type _ubar;
  mushrTypes::Ubar::params _params_ubar_u;

  double _idle;

  double _wheelbase;
};  // namespace mushr

// X_j = X_i + \dpt{x}_i * dt
class mushr_x_xdot_t
{
public:
  using X = mushrTypes::State::type;
  using Xdot = mushrTypes::StateDot::type;

  mushr_x_xdot_t() : _dt(prx::simulation_step)
  {
  }

  static X predict(const X& x, const Xdot& xdot, const double dt)
  {
    // const gtsam::Pose2 res{ gtsam::Pose2(x[0], x[1], x[2]) * gtsam::Pose2::Expmap(xdot * dt) };
    // return X{ res.x(), res.y(), res.theta() };
    return x + xdot * dt;
  }

  virtual X compute_error(const X& xi, const Xdot& xdot, const X& xj) const
  {
    const X prediction{ predict(xi, xdot, _dt) };
    X error{ prediction - xj };
    error[2] = angle_diff(prediction[2], xj[2]);
    return error;
  }

private:
  const double _dt;
};

class mushr_x_xdot_ub_t
{
public:
  using X = mushrTypes::State::type;
  using Xdot = mushrTypes::StateDot::type;
  using Ubar = mushrTypes::Ubar::type;

  static Xdot predict(const X& x, const Ubar& ubar)
  {
    const double& vt{ ubar[mushrTypes::Ubar::velocity] };
    const double& beta{ ubar[mushrTypes::Ubar::beta] };

    const double& theta{ x[mushrTypes::State::theta] };

    const double cTh{ std::cos(theta + beta) };
    const double sTh{ std::sin(theta + beta) };
    const double wt{ 2.0 * vt * std::sin(beta) / mushrTypes::Parameters::L };
    return Xdot{
      vt * cTh,  // no-indent
      vt * sTh,  // no-indent
      wt         // no-indent
    };
  }

  virtual Xdot compute_error(const X& xi, const Xdot& xdot, const Ubar& ub) const
  {
    return predict(xi, ub) - xdot;
  }

private:
};

class mushr_ub_u_xdot_param_t
{
public:
  using Xdot = mushrTypes::StateDot::type;
  using Ubar = mushrTypes::Ubar::type;
  using Params = mushrTypes::Ubar::params;
  using U = mushrTypes::Control::type;

  mushr_ub_u_xdot_param_t()
  {
  }

  static Ubar predict(const U& u, const Ubar& ubar, const Params& params)
  {
    const double& v_current{ ubar[mushrTypes::Ubar::velocity] };

    const double& steering{ u[mushrTypes::Control::steering] };
    const double& v_desired{ u[mushrTypes::Control::vel_desired] };

    const double& accel_slope{ params[mushrTypes::Ubar::accel_slope] };

    const double dv{ v_desired - v_current };
    const double v_next{ v_current + dv * accel_slope };

    const double beta{ std::atan(0.5 * std::tan(steering)) };

    Ubar ubar_next{};
    ubar_next[mushrTypes::Ubar::beta] = beta;
    ubar_next[mushrTypes::Ubar::velocity] = v_next;
    return ubar_next;
  }

  virtual Ubar compute_error(const Ubar& ubar1, const U& u, const Ubar& ubar0, const Params& params) const
  {
    return predict(u, ubar0, params) - ubar1;
  }

private:
};

}  // namespace prx
PRX_REGISTER_SYSTEM(mushrFG_t, mushr)
