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
  mushrTypes::State::type _state;
  mushrTypes::StateDot::type _state_dot;
  mushrTypes::Control::type _ctrl;
  mushrTypes::Ubar::type _ubar;
  mushrTypes::Ubar::params _params_ubar_u;

  double _idle;
};  // namespace mushr

// X_j = X_i + \dpt{x}_i * dt
class mushr_x_xdot_t : public noise_model_3factor_t<3, 3, 3>
{
  using Base = noise_model_3factor_t<3, 3, 3>;

public:
  using X = mushrTypes::State::type;
  using Xdot = mushrTypes::StateDot::type;

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
    const X prediction{ predict(xi, xdot, _dt) };
    X error{ prediction - xj };
    error[2] = angle_diff(prediction[2], xj[2]);
    return error;
  }

  void eval_to_stream(gtsam::Values& values, std::ostream& os)
  {
    const X xi{ values.at<X>(key<1>()) };
    const Xdot xdot{ values.at<Xdot>(key<2>()) };
    const X xj{ values.at<X>(key<3>()) };

    os << prx::fg::symbol_factory_t::formatter(key<1>()) << " " << xi.transpose() << " ";    // 1, 2, 3
    os << prx::fg::symbol_factory_t::formatter(key<2>()) << " " << xdot.transpose() << " ";  // 4, 5, 6
    os << prx::fg::symbol_factory_t::formatter(key<3>()) << " " << xj.transpose() << " ";    // 7, 8, 9
    os << "dt " << _dt << " ";                                                               // 10
    os << "Error " << compute_error(xi, xdot, xj).transpose() << " ";                        // 4, 5, 6
    os << "\n";
  }

private:
  const double _dt;
};

class mushr_x_xdot_ub_t : public noise_model_3factor_t<3, 3, mushrTypes::Ubar::Dim>
{
  using Base = noise_model_3factor_t<3, 3, mushrTypes::Ubar::Dim>;

public:
  using X = mushrTypes::State::type;
  using Xdot = mushrTypes::StateDot::type;
  using Ubar = mushrTypes::Ubar::type;

  mushr_x_xdot_ub_t(gtsam::Key x, gtsam::Key xdot, gtsam::Key ubar,
                    const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(x, xdot, ubar, cost_model)
  {
  }

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

  virtual Xdot compute_error(const X& xi, const Xdot& xdot, const Ubar& ub) const override
  {
    return predict(xi, ub) - xdot;
  }

  void eval_to_stream(gtsam::Values& values, std::ostream& os)
  {
    const X xi{ values.at<X>(key<1>()) };
    const Xdot xdot{ values.at<Xdot>(key<2>()) };
    const Ubar ubar{ values.at<Ubar>(key<3>()) };

    os << prx::fg::symbol_factory_t::formatter(key<1>()) << " " << xi.transpose() << " ";    // 1, 2, 3
    os << prx::fg::symbol_factory_t::formatter(key<2>()) << " " << xdot.transpose() << " ";  // 4, 5, 6
    os << prx::fg::symbol_factory_t::formatter(key<3>()) << " " << ubar.transpose() << " ";  // 7, 8
    os << "Error: " << compute_error(xi, xdot, ubar).transpose() << " ";                     // 9, 10, 11
    os << "\n";
  }

private:
};

class mushr_ub_u_xdot_param_t
  : public noise_model_4factor_t<mushrTypes::Ubar::Dim, 2, mushrTypes::Ubar::Dim, mushrTypes::Ubar::ParamsDim>
{
  using Base = noise_model_4factor_t<mushrTypes::Ubar::Dim, 2, mushrTypes::Ubar::Dim, mushrTypes::Ubar::ParamsDim>;

public:
  using Xdot = mushrTypes::StateDot::type;
  using Ubar = mushrTypes::Ubar::type;
  using Params = mushrTypes::Ubar::params;
  using U = mushrTypes::Control::type;

  mushr_ub_u_xdot_param_t(gtsam::Key ubar, gtsam::Key u, gtsam::Key xdot, gtsam::Key param,
                          const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(ubar, u, xdot, param, cost_model)
  {
  }

  static Ubar predict(const U& u, const Ubar& ubar, const Params& params)
  {
    const double& v_current{ ubar[mushrTypes::Ubar::velocity] };
    const double& steering{ u[mushrTypes::Control::steering] };
    const double& v_desired{ u[mushrTypes::Control::vel_desired] };

    const double& accel_slope{ params[mushrTypes::Ubar::accel_slope] };
    const double& steering_param{ params[mushrTypes::Ubar::steering_param] };
    const double& max_vel_param{ params[mushrTypes::Ubar::max_vel_param] };

    const double dv{ v_desired - v_current };
    const double v_next{ v_current + dv * accel_slope };
    const double beta{ std::atan(0.5 * std::tan(steering * steering_param)) };

    Ubar ubar_next{};
    ubar_next[mushrTypes::Ubar::beta] = beta;
    ubar_next[mushrTypes::Ubar::velocity] = max_vel_param * v_next;

    return ubar_next;
  }

  virtual Ubar compute_error(const Ubar& ubar1, const U& u, const Ubar& ubar0, const Params& params) const
  {
    return predict(u, ubar0, params) - ubar1;
  }

  void eval_to_stream(gtsam::Values& values, std::ostream& os)
  {
    const Ubar ubar1{ values.at<Ubar>(key<1>()) };
    const U u{ values.at<U>(key<2>()) };
    const Ubar Ubar0{ values.at<Ubar>(key<3>()) };
    const Params params{ values.at<Params>(key<4>()) };

    os << ubar1.transpose() << " ";                                   // 1, 2, 3
    os << u.transpose() << " ";                                       // 4, 5, 6
    os << Ubar0.transpose() << " ";                                   // 7, 8
    os << params.transpose() << " ";                                  // 9
    os << compute_error(ubar1, u, Ubar0, params).transpose() << " ";  // 4, 5, 6
    os << "\n";
  }

private:
};

class mushr_x_observation_t : public noise_model_1factor_t<3>
{
  using Base = noise_model_1factor_t<3>;

public:
  using X = mushrTypes::State::type;

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

class mushr_x_async_observation_t : public noise_model_2factor_t<3, 3>
{
  using Base = noise_model_2factor_t<3, 3>;

public:
  using X = Base::X0;

  mushr_x_async_observation_t(const gtsam::Key xi, const gtsam::Key xj, const X z, double z_ti,
                              const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(xi, xj, cost_model), _z(z), _ti(z_ti)
  {
    prx_assert(0 <= _ti and _ti <= 1, "Time of observation [" << _ti << "] outside of the interval");
  }
  mushr_x_async_observation_t(const gtsam::Key xi, const gtsam::Key xj, const X z, double t_xi, double t_xj,
                              double t_zi, const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : mushr_x_async_observation_t(xi, xj, z, (t_zi - t_xi) / (t_xj - t_xi), cost_model)
  {
  }

  static X predict(const X& xi, const X& xj, const double ti)
  {
    const X xij{ (1.0 - ti) * xi + ti * xj };
    const double theta_i{ xi[2] };
    const double theta_j{ xj[2] };
    double theta{ 0.0 };
    // PRX_DEBUG_VAR_1("-: -: -: -: -: -: -: -: -: -: -: -: -");
    // PRX_DEBUG_VAR_3(xi.transpose(), xj.transpose(), xij.transpose());
    if (std::fabs(theta_i - theta_j) < prx::constants::pi)
    {
      theta = xij[2];
    }
    else
    {
      if (theta_i < theta_j)
        theta = theta_j + (1.0 - ti) * (theta_i - theta_j + 2.0 * prx::constants::pi);
      else
        theta = theta_i + ti * (2.0 * prx::constants::pi - theta_i + theta_j);
    }
    // theta = norm_angle_pi(theta);
    // PRX_DEBUG_VAR_1(theta);
    return X(xij[0], xij[1], theta);
  }

  virtual X compute_error(const X& xi, const X& xj) const override
  {
    const X prediction{ mushr_x_async_observation_t::predict(xi, xj, _ti) };
    const X xp{ prediction - _z };
    const double theta{ angle_diff(prediction[2], _z[2]) };

    return X(xp[0], xp[1], theta);
  }

  void eval_to_stream(gtsam::Values& values, std::ostream& os)
  {
    const X xi{ values.at<X>(key<1>()) };
    const X xj{ values.at<X>(key<2>()) };

    os << prx::fg::symbol_factory_t::formatter(key<1>()) << " " << xi.transpose() << " ";  // 1, 2, 3
    os << prx::fg::symbol_factory_t::formatter(key<2>()) << " " << xj.transpose() << " ";  // 4, 5, 6
    os << "z " << _z.transpose() << " ";                                                   // 7, 8, 9
    os << "t " << _ti << " ";                                                              // 10
    os << "Error " << compute_error(xi, xj).transpose() << " ";                            // 11, 12, 13
    os << "\n";
  }

protected:
  const X _z;
  const double _ti;
};

inline void add_mushr_factor_graph_step(const std::size_t t, const double dt, gtsam::NonlinearFactorGraph& graph,
                                        const mushrConfig& config, const std::size_t traj_idx = 0)
{
  using namespace mushrTypes;
  graph.emplace_shared<mushr_x_xdot_t>(k_X(t, traj_idx), k_Xd(t, traj_idx), k_X(t + 1, traj_idx), dt, config.cm_x_xdot);
  graph.emplace_shared<mushr_x_xdot_ub_t>(k_X(t, traj_idx), k_Xd(t + 1, traj_idx), k_Ub(t, traj_idx),
                                          config.cm_x_xdot_ub);
  // graph.emplace_shared<mushr_ub_u_xdot_t>(k_Ub(t), k_U(t), k_Xd(t), config.length, config.cm_ub_u);
  graph.emplace_shared<mushr_ub_u_xdot_param_t>(k_Ub(t + 1, traj_idx), k_U(t, traj_idx), k_Ub(t, traj_idx), k_Ps("dv"),
                                                config.cm_ub_u);
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
