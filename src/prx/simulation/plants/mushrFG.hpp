#pragma once
#include <array>
#include <numeric>

#include "prx/simulation/plant.hpp"
#include "prx/utilities/math/first_order_derivative.hpp"

namespace prx
{
inline double angle_diff(const double a, const double b)
{
  return std::atan2(std::sin(a - b), std::cos(a - b));
}

namespace mushrTypes
{

using State = Eigen::Vector<double, 3>;
using StateDot = Eigen::Vector<double, 3>;
using Control = Eigen::Vector<double, 2>;
using Parameters = Eigen::Vector<double, 6>;
using ParamsUbarU = Eigen::Vector<double, 3>;

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
static inline double steering(const Control& u)
{
  return u[0];
}
static inline double steering(const Control& u, const Parameters& params)
{
  const double us{ u[0] * steering_gain(params) + steering_offset(params) };
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
class mushrFG_t : public prx::plant_t
{
public:
  mushrFG_t(const std::string& path);
  ~mushrFG_t();
  virtual void propagate(const double simulation_step) override final;
  virtual void update_configuration() override;
  virtual void compute_derivative() override final;

protected:
  mushrTypes::State _state;
  mushrTypes::StateDot _state_dot;
  mushrTypes::Control _ctrl;
  mushrTypes::Parameters _params;
  mushrTypes::Control _ubar;
  mushrTypes::ParamsUbarU _params_ubar_u;
  double _length;
};  // namespace mushr

// X_j = X_i + \dpt{x}_i * dt
class mushr_x_xdot_t
{
public:
  using X = Eigen::Vector<double, 3>;
  using Xdot = Eigen::Vector<double, 3>;

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
  using X = Eigen::Vector<double, 3>;
  using Xdot = Eigen::Vector<double, 3>;
  using Ubar = Eigen::Vector<double, 2>;

  static Xdot predict(const X& x, const Ubar& ubar)
  {
    const double cTh{ std::cos(x[2]) };  // cos(theta)
    const double sTh{ std::sin(x[2]) };  // sin(theta)
    const double& vt{ ubar[0] };
    const double& wt{ ubar[1] };
    return Xdot{
      cTh * vt,  // no-indent
      sTh * vt,  // no-indent
      wt         // no-indent
    };
  }

  virtual Xdot compute_error(const X& xi, const Xdot& xdot, const Ubar& ub) const
  {
    return predict(xi, ub) - xdot;
  }

private:
};

class mushr_ub_u_xdot_t
{
public:
  using Xdot = Eigen::Vector<double, 3>;
  using Params = Eigen::Vector<double, 3>;
  using Ubar = Eigen::Vector<double, 2>;
  using U = Eigen::Vector<double, 2>;

  mushr_ub_u_xdot_t(double length) : _length(length)
  {
  }

  static Ubar predict(const U& u, const Xdot& xdot, const double& length)
  {
    const double vt{ xdot.head(2).norm() };  // \sqrt(\dot{x} + \dot{y})
    const double dv_cap{ mushrTypes::desired_velocity(u) };
    const double w{ vt * (std::tan(mushrTypes::steering(u)) / length) };
    return Ubar(dv_cap, w);
  }

  virtual Ubar compute_error(const Ubar& ub, const U& u, const Xdot& xdot) const
  {
    return predict(u, xdot, _length) - ub;
  }

private:
  const double _length;
};

class mushr_ub_u_xdot_param_t
{
public:
  using Xdot = Eigen::Vector<double, 3>;
  using Params = Eigen::Vector<double, 3>;
  using Ubar = Eigen::Vector<double, 2>;
  using U = Eigen::Vector<double, 2>;

  mushr_ub_u_xdot_param_t(double length) : _length(length)
  {
  }

  static Ubar predict(const U& u, const Xdot& xdot, const Params& params, const double& length)
  {
    const double slope_pos{ mushrTypes::positive_slope(params) };
    const double steering_offset{ mushrTypes::steering_offset(params) };
    const double velocity_gain{ mushrTypes::velocity_gain(params) };

    const double vt{ xdot.head(2).norm() };  // \sqrt(\dot{x} + \dot{y})
    const double dv{ mushrTypes::desired_velocity(u) - vt };
    // const double dv_cap{ vt + std::max(std::min(dv, 0.1), -0.1) };
    const double dv_cap{ vt + dv * slope_pos };
    // const double dv_cap{ mushrTypes::desired_velocity(u) * slope_pos };
    const double w{ vt * (std::tan(mushrTypes::steering(u) + steering_offset) / length) };
    // PRX_DEBUG_VAR_3(vt, dv, dv_cap);
    return Ubar(dv_cap, w * velocity_gain);
  }

  virtual Ubar compute_error(const Ubar& ub, const U& u, const Xdot& xdot, const Params& params) const
  {
    return predict(u, xdot, params, _length) - ub;
  }

private:
  const double _length;
};

}  // namespace prx
PRX_REGISTER_SYSTEM(prx::mushrFG_t, mushrFG)
