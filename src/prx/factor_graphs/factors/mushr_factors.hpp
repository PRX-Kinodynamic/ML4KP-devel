#pragma once
#include <array>
#include <numeric>
#include <gtsam/config.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "prx/simulation/plant.hpp"
#include "prx/utilities/math/first_order_derivative.hpp"
#include "prx/factor_graphs/factors/noise_model_factor.hpp"
#include "prx/factor_graphs/utilities/symbols_factory.hpp"
#include "prx/factor_graphs/factors/mushr_types.hpp"
// #include "prx/factor_graphs/factors/noise_model_factor.hpp"
// #include "prx/factor_graphs/utilities/perception/camera.hpp"
// #include "prx/factor_graphs/utilities/symbols_factory.hpp"

namespace prx
{
namespace fg
{
struct mushrConfig
{
  double length;
  gtsam::SharedNoiseModel cm_x_xdot;
  gtsam::SharedNoiseModel cm_x_xdot_ub;
  gtsam::SharedNoiseModel cm_ub_u;
  gtsam::SharedNoiseModel cm_x_z;
};

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
  mushrTypes::ParamsDeltaVel _params_dv;
  double _length;
};  // namespace mushr

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
    _g(1, 0) = std::cos(mushrTypes::theta(state));
    _g(2, 0) = std::sin(mushrTypes::theta(state));
    const double vel_delta{ mushrTypes::desired_velocity(u, params) - mushrTypes::current_velocity(state) };
    // const double vel_delta_cap{ std::max(std::min(vel_delta, vel_delta_max(params)), vel_delta_min(params)) };
    const double vel_delta_cap{ mushrTypes::bound(vel_delta, mushrTypes::vel_delta_min(params),
                                                  mushrTypes::vel_delta_max(params)) };
    const double w{ mushrTypes::current_velocity(state) * std::tan(mushrTypes::steering(u, params)) / _length };
    return (State() << _g * Eigen::Vector2d(mushrTypes::current_velocity(state), w), vel_delta).finished();
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
    os << p.transpose() << " ";
    os << "\n";  // 12, 13, 14, 15, 16, 17
  }

private:
  mutable Eigen::Matrix<double, 3, 2> _g;
  const double _dt;
  const double _length;
};

// X_j = X_i + \dpt{x}_i * dt
class mushr_x_xdot_t : public noise_model_3factor_t<3, 3, 3>
{
  using Base = noise_model_3factor_t<3, 3, 3>;

public:
  using X = Base::X0;
  // using State = Base::X1;
  using Xdot = Base::X2;

  mushr_x_xdot_t(gtsam::Key xi, gtsam::Key xdot, gtsam::Key xj, const double dt,
                 const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(xi, xdot, xj, cost_model), _dt(dt)
  {
  }

  static X predict(const X& x, const Xdot& xdot, const double dt)
  {
    // const gtsam::Pose2 res{ gtsam::Pose2(x[0], x[1], x[2]) * gtsam::Pose2::Expmap(xdot * dt) };
    // return X{ res.x(), res.y(), res.theta() };
    return x + xdot * dt;
  }

  virtual X compute_error(const X& xi, const Xdot& xdot, const X& xj) const override
  {
    return predict(xi, xdot, _dt) - xj;
  }

  void eval_to_stream(gtsam::Values& values, std::ostream& os)
  {
    const X xi{ values.at<X>(key<1>()) };
    const Xdot xdot{ values.at<Xdot>(key<2>()) };
    const X xj{ values.at<X>(key<3>()) };

    os << xi.transpose() << " ";                           // 1, 2, 3
    os << xdot.transpose() << " ";                         // 4, 5, 6
    os << xj.transpose() << " ";                           // 7, 8, 9
    os << _dt << " ";                                      // 10
    os << compute_error(xi, xdot, xj).transpose() << " ";  // 4, 5, 6
    os << "\n";
  }

private:
  const double _dt;
};

class mushr_x_xdot_ub_t : public noise_model_3factor_t<3, 3, 2>
{
  using Base = noise_model_3factor_t<3, 3, 2>;

public:
  using X = Base::X0;
  using Xdot = Base::X1;
  using Ubar = Base::X2;

  mushr_x_xdot_ub_t(gtsam::Key x, gtsam::Key xdot, gtsam::Key ubar,
                    const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(x, xdot, ubar, cost_model)
  {
  }

  static Xdot predict(const X& x, const Ubar& u)
  {
    const double cTh{ std::cos(x[2]) };  // cos(theta)
    const double sTh{ std::sin(x[2]) };  // sin(theta)
    const double& vt{ u[0] };
    const double& wt{ u[1] };
    return Xdot{
      cTh * vt,  // no-indent
      sTh * vt,  // no-indent
      wt         // no-indent
    };           // no-indent
  }

  virtual Xdot compute_error(const X& xi, const Xdot& xdot, const Ubar& ub) const override
  {
    return predict(xi, ub) - xdot;
  }

  void eval_to_stream(gtsam::Values& values, std::ostream& os)
  {
    const X xi{ values.at<X>(key<1>()) };
    const Xdot xdot{ values.at<Xdot>(key<2>()) };
    const Ubar ubar{ values.at<Ubar>(key<3>()) };

    os << xi.transpose() << " ";                             // 1, 2, 3
    os << xdot.transpose() << " ";                           // 4, 5, 6
    os << ubar.transpose() << " ";                           // 7, 8
    os << compute_error(xi, xdot, ubar).transpose() << " ";  // 4, 5, 6
    os << "\n";
  }

private:
};

class mushr_ub_u_xdot_t : public noise_model_4factor_t<2, 2, 3, 1>
{
  using Base = noise_model_4factor_t<2, 2, 3, 1>;

public:
  using Ubar = Base::X0;
  using U = Base::X1;
  using Xdot = Base::X2;
  using Params = Base::X3;

  mushr_ub_u_xdot_t(gtsam::Key ubar, gtsam::Key u, gtsam::Key xdot, gtsam::Key param, double length,
                    const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(ubar, u, xdot, param, cost_model), _length(length)
  {
  }

  static Ubar predict(const U& u, const Xdot& xdot, const Params& params, const double& length)
  {
    const double slope_pos{ mushrTypes::positive_slope(params) };

    const double vt{ xdot.head(2).norm() };  // \sqrt(\dot{x} + \dot{y})
    const double dv{ mushrTypes::desired_velocity(u) - vt };
    // const double dv_cap{ vt + std::max(std::min(dv, 0.1), -0.1) };
    const double dv_cap{ vt + dv * slope_pos };
    const double w{ vt * (std::tan(mushrTypes::steering(u)) / length) };
    return Ubar(dv_cap, w);
  }

  virtual Ubar compute_error(const Ubar& ub, const U& u, const Xdot& xdot, const Params& params) const override
  {
    return predict(u, xdot, params, _length) - ub;
  }

  void eval_to_stream(gtsam::Values& values, std::ostream& os)
  {
    const Ubar ubar{ values.at<Ubar>(key<1>()) };
    const U u{ values.at<U>(key<2>()) };
    const Xdot xdot{ values.at<Xdot>(key<3>()) };
    const Params params{ values.at<Params>(key<4>()) };

    os << ubar.transpose() << " ";                                  // 1, 2, 3
    os << u.transpose() << " ";                                     // 4, 5, 6
    os << xdot.transpose() << " ";                                  // 7, 8
    os << params.transpose() << " ";                                // 9
    os << compute_error(ubar, u, xdot, params).transpose() << " ";  // 4, 5, 6
    os << "\n";
  }

private:
  mutable Eigen::Matrix<double, 3, 2> _g;
  const double _length;
};

class mushr_x_observation_t : public noise_model_1factor_t<3>
{
  using Base = noise_model_1factor_t<3>;

public:
  using X = Base::X0;
  using Z = Base::X1;

  mushr_x_observation_t(const gtsam::Key x, const X z, const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(x, cost_model), _z(z)
  {
  }

  virtual X compute_error(const X& x) const override
  {
    X error{ x - _z };
    error[2] = angle_diff(x[2], _z[2]);
    return error;
  }

  void eval_to_stream(gtsam::Values& values, std::ostream& os)
  {
    const X x{ values.at<X>(key<1>()) };

    os << x.transpose() << " ";                 // 1, 2, 3
    os << _z.transpose() << " ";                // 4, 5, 6
    os << compute_error(x).transpose() << " ";  // 4, 5, 6
    os << "\n";
  }

protected:
  const X _z;
};

inline void add_mushr_factor_graph_step(const std::size_t t, const double dt, gtsam::NonlinearFactorGraph& graph,
                                        const mushrConfig& config)
{
  using namespace mushrTypes;
  graph.emplace_shared<mushr_x_xdot_t>(k_X(t), k_Xd(t), k_X(t + 1), dt, config.cm_x_xdot);
  graph.emplace_shared<mushr_x_xdot_ub_t>(k_X(t), k_Xd(t), k_Ub(t), config.cm_x_xdot_ub);
  graph.emplace_shared<mushr_ub_u_xdot_t>(k_Ub(t), k_U(t), k_Xd(t), k_Ps("dv"), config.length, config.cm_ub_u);
}

// void fwd_propagate(const mushr::State& X0, const mushr::StateDot& Xdot0, const prx::plan_t& plan,
//                    const prx::trajectory_t& traj)
// {
// traj.clear();
// traj.push_back(X0);
// for (auto plan_step : plan)
// {
//   const mushr::Control u{ plan_step.control->as<Control>() };
//   for (double ti = 0; ti < plan_step.duration; ti += dt)
//   {
//     const mushr::StateDot xdot{};
//     const mushr::State x{ traj.back() };
//     const X x{ mushr_x_xdot_t::predict(x, const Xdot& xdot, dt) };
//     const Xdot xdot{ mushr_x_xdot_ub_t::predict(x, u) };
//     const Ubar ubar{ mushr_ub_u_xdot_t::predict(u, const Xdot& xdot) };
//   }
// }
// }

}  // namespace fg
}  // namespace prx
PRX_REGISTER_SYSTEM(fg::mushrFG_t, mushrFG)
