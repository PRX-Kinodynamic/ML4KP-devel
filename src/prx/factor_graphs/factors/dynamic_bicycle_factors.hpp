#pragma once
#include <array>
#include <numeric>
#include <gtsam/config.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "prx/factor_graphs/utilities/gtsam_extra_utilities.hpp"
#include "prx/factor_graphs/factors/euler_integration_factor.hpp"

namespace prx
{
namespace fg
{
// Model from:
// Kabzan, Juraj, Lukas Hewing, Alexander Liniger, and Melanie N. Zeilinger.
// "Learning-based model predictive control for autonomous racing."
// IEEE Robotics and Automation Letters 4, no. 4 (2019): 3363-3370.
struct bike_dynamics_t
{
public:
  static constexpr Eigen::Index DimCtrl{ 2 };
  static constexpr Eigen::Index DimVel{ 3 };
  static constexpr Eigen::Index DimParams{ 8 };
  static constexpr Eigen::Index DimStaticParams{ 3 };

  using Velocity = Eigen::Vector3d;
  using Acceleration = Eigen::Vector3d;
  using Control = Eigen::Vector<double, 2>;
  using Vector1D = Eigen::Vector<double, 1>;

  using StaticParams = Eigen::Vector<double, DimStaticParams>;
  using Params = Eigen::Vector<double, DimParams>;

  template <Eigen::Index DimOut, Eigen::Index DimIn>
  using OptJacobian = gtsam::OptionalJacobian<DimOut, DimIn>;

  struct indices
  {
    // Controls
    const static uint8_t thrust{ 0 };
    const static uint8_t delta{ 1 };

    // Parameters
    // Increase 1 from previous for easier index management. Change to enum?
    const static uint8_t pacejka_B{ 0 };
    const static uint8_t pacejka_C{ pacejka_B + 1 };
    const static uint8_t pacejka_D{ pacejka_C + 1 };
    // const static uint8_t Croll{ 3 };
    const static uint8_t Cr0{ pacejka_D + 1 };
    const static uint8_t Cr2{ Cr0 + 1 };
    const static uint8_t Ptv{ Cr2 + 1 };
    const static uint8_t Iz{ Ptv + 1 };

    // Static params aka those that can be known directly and need no estimation
    const static uint8_t mass{ 0 };
    const static uint8_t length_rear{ 1 };
    const static uint8_t length_front{ 2 };
  };

  static StaticParams init_static_params(const double mass, const double length_rear, const double length_front)
  {
    StaticParams sp;
    sp[indices::mass] = mass;
    sp[indices::length_rear] = length_rear;
    sp[indices::length_front] = length_front;
    return sp;
  }

  static Vector1D slip_angle(const Velocity& xdot, const double& delta, const char FR,
                             const StaticParams& static_params,  // no-lint
                             OptJacobian<1, 3> Hxdot = boost::none, OptJacobian<1, 1> Hdelta = boost::none)
  {
    const double& vx{ xdot[0] };
    const double& vy{ xdot[1] };
    const double& r{ xdot[2] };

    prx_assert(FR == 'F' or FR == 'R', "FR can only be F or R");
    const double& lR{ static_params[indices::length_rear] };
    const double& lF{ static_params[indices::length_front] };

    const double d{ FR == 'F' ? delta : 0.0 };
    const double lrf{ FR == 'F' ? lF * r : -lR * r };

    const double alpha{ std::atan2(vy + lrf, vx) - d };
    // PRX_DBG_VARS(r, lrf, alpha);

    if (Hxdot)
    {
      const double alpha_H_lrf{ 1.0 / (vx * (std::pow(lrf + vy, 2) / (vx * vx) + 1)) };
      const double lrf_H_r{ FR == 'F' ? lF : -lR };

      const double Hvx{ -(lrf + vy) / (vx * vx * (std::pow(lrf + vy, 2) / (vx * vx) + 1)) };
      const double Hvy{ 1.0 / (vx * (std::pow(lrf + vy, 2) / (vx * vx) + 1)) };
      // const double Hvx{ (lrf + lrf) / (std::pow(vx, 2) * (std::pow(lrf - vy, 2) / std::pow(vx, 2) + 1)) };
      // const double Hvy{ 1.0 / (vx * (std::pow(lrf + lrf, 2) / std::pow(vx, 2) + 1)) };
      const double Hr{ alpha_H_lrf * lrf_H_r };

      *Hxdot << Hvx, Hvy, Hr;
      // PRX_DBG_VARS(*Hxdot);
    }
    if (Hdelta)
    {
      *Hdelta << (FR == 'F' ? -1.0 : 0.0);
    }

    return Vector1D(alpha);
  }

  static Vector1D lateral_force(const Velocity& xdot, const double& delta, const Params& params, const char FR,
                                const StaticParams& static_params,       // no-lint
                                OptJacobian<1, 3> Hxdot = boost::none,   // no-lint
                                OptJacobian<1, 1> Hdelta = boost::none,  // no-lint
                                OptJacobian<1, DimParams> Hparams = boost::none)
  {
    Eigen::Matrix<double, 1, 3> zero13{ Eigen::Matrix<double, 1, 3>::Zero() };
    Eigen::Matrix<double, 1, 1> zero11{ Eigen::Matrix<double, 1, 1>::Zero() };

    OptJacobian<1, 3> alpha_H_xdot{ init_optional_jacobian(zero13, Hxdot) };
    OptJacobian<1, 1> alpha_H_delta{ init_optional_jacobian(zero11, Hxdot) };

    // Eigen::Matrix<double, 1, 3> alpha_H_xdot{ Eigen::Matrix<double, 1, 3>::Zero() };
    // Eigen::Matrix<double,1, 1> alpha_H_delta{Eigen::Matrix<double,1, 1> ::Zero()};

    const double B{ params[indices::pacejka_B] };
    const double C{ params[indices::pacejka_C] };
    const double D{ params[indices::pacejka_D] };

    const double alpha{ slip_angle(xdot, delta, FR, static_params, alpha_H_xdot, alpha_H_delta)[0] };

    // if (alpha_H_xdot)
    //   PRX_DBG_VARS(*alpha_H_xdot)
    const Vector1D res{ Vector1D(D * std::sin(C * std::atan(B * alpha))) };

    if (Hxdot or Hdelta)
    {
      const double d_res_alpha{ (B * C * D * std::cos(C * std::atan(B * alpha))) / (B * B * alpha * alpha + 1) };
      const Eigen::Matrix<double, 1, 1> res_H_alpha{ { d_res_alpha } };
      // PRX_DBG_VARS(*alpha_H_xdot);
      // PRX_DBG_VARS(res_H_alpha);
      if (Hxdot)
        *Hxdot = res_H_alpha * (*alpha_H_xdot);
      if (Hdelta)
        *Hdelta = res_H_alpha * (*alpha_H_delta);
    }
    if (Hparams)
    {
      const double dres_B{ (C * D * alpha * std::cos(C * std::atan(B * alpha))) / (B * B * alpha * alpha + 1) };
      const double dres_C{ D * std::atan(B * alpha) * std::cos(C * std::atan(B * alpha)) };
      const double dres_D{ std::sin(C * std::atan(B * alpha)) };
      Eigen::Matrix<double, 1, DimParams> deriv{ Eigen::Matrix<double, 1, DimParams>::Zero() };
      deriv(0, indices::pacejka_B) = dres_B;
      deriv(0, indices::pacejka_C) = dres_C;
      deriv(0, indices::pacejka_D) = dres_D;
      *Hparams = deriv;
    }

    return res;
  }

  static Vector1D longitudinal_force(const Velocity& xdot, const Control& u, const Params& params,  // no-lint
                                     OptJacobian<1, DimVel> Hxdot = boost::none,                    // no-lint
                                     OptJacobian<1, DimCtrl> Hu = boost::none,                      // no-lint
                                     OptJacobian<1, DimParams> Hparams = boost::none)
  {
    const double& vx{ xdot[0] };

    const double& thrust{ u[indices::thrust] };

    const double& Croll{ 1.0 };
    // const double& Croll{ params[indices::Croll] };
    const double& Cr0{ params[indices::Cr0] };
    const double& Cr2{ params[indices::Cr2] };  // drag

    const double Fx{ thrust - Cr0 - Cr2 * vx * vx };

    if (Hxdot)
    {
      const double dvx{ -2.0 * Cr2 * vx };
      *Hxdot << dvx, 0.0, 0.0;
    }
    if (Hu)
    {
      const double dthrust{ Croll };
      *Hu = Eigen::Matrix<double, 1, DimCtrl>::Zero();
      (*Hu)(0, indices::thrust) = dthrust;
    }
    if (Hparams)
    {
      const double dCroll{ thrust };
      const double dCr0{ -1.0 };
      const double dCr2{ -vx * vx };

      Eigen::Matrix<double, 1, DimParams> deriv{ Eigen::Matrix<double, 1, DimParams>::Zero() };
      // deriv(0, indices::Croll) = dCroll;
      deriv(0, indices::Cr0) = dCr0;
      deriv(0, indices::Cr2) = dCr2;

      *Hparams = deriv;
    }
    return Vector1D(Fx);
  }

  static Vector1D torque_vectoring_controller(const double& delta, const Velocity& xdot, const Params& params,
                                              const StaticParams& static_params,           // no-lint
                                              OptJacobian<1, 1> Hdelta = boost::none,      // no-lint
                                              OptJacobian<1, DimVel> Hxdot = boost::none,  // no-lint
                                              OptJacobian<1, DimParams> Hparams = boost::none)
  {
    const double& vx{ xdot[0] };
    const double& r{ xdot[2] };

    const double& lR{ static_params[indices::length_rear] };
    const double& lF{ static_params[indices::length_front] };

    const double& Ptv{ params[indices::Ptv] };

    const double r_target{ delta * vx / (lR + lF) };
    const double tau_TV{ (r_target - r) * Ptv };

    if (Hdelta)
    {
      const double ddelta{ (Ptv * vx) / (lF + lR) };
      *Hdelta << ddelta;
    }
    if (Hxdot)
    {
      const double dvx{ (Ptv * delta) / (lF + lR) };
      const double dr{ -Ptv };
      *Hxdot << dvx, 0.0, dr;
    }
    if (Hparams)
    {
      const double dPtv{ (delta * vx) / (lF + lR) - r };
      Eigen::Matrix<double, 1, DimParams> deriv{ Eigen::Matrix<double, 1, DimParams>::Zero() };
      deriv(0, indices::Ptv) = dPtv;

      *Hparams = deriv;
    }

    return Vector1D(tau_TV);
  }

  static Eigen::Vector3d acceleration(const Velocity& xdot, const Control& u, const Params& params,
                                      const StaticParams& static_params,            // no-lint
                                      OptJacobian<3, DimVel> Hxdot = boost::none,   // no-lint
                                      OptJacobian<3, DimCtrl> Hctrl = boost::none,  // no-lint
                                      OptJacobian<3, DimParams> Hparams = boost::none)
  {
    Eigen::Matrix<double, 1, DimVel> Ffy_zero_vel{ Eigen::Matrix<double, 1, DimVel>::Zero() };
    Eigen::Matrix<double, 1, 1> Ffy_zero_delta{ Eigen::Matrix<double, 1, 1>::Zero() };
    Eigen::Matrix<double, 1, DimParams> Ffy_zero_params{ Eigen::Matrix<double, 1, DimParams>::Zero() };

    Eigen::Matrix<double, 1, DimVel> Fry_zero_vel{ Eigen::Matrix<double, 1, DimVel>::Zero() };
    Eigen::Matrix<double, 1, 1> Fry_zero_delta{ Eigen::Matrix<double, 1, 1>::Zero() };
    Eigen::Matrix<double, 1, DimParams> Fry_zero_params{ Eigen::Matrix<double, 1, DimParams>::Zero() };

    Eigen::Matrix<double, 1, DimVel> tauTV_zero_vel{ Eigen::Matrix<double, 1, DimVel>::Zero() };
    Eigen::Matrix<double, 1, 1> tauTV_zero_delta{ Eigen::Matrix<double, 1, 1>::Zero() };
    Eigen::Matrix<double, 1, DimParams> tauTV_zero_params{ Eigen::Matrix<double, 1, DimParams>::Zero() };

    Eigen::Matrix<double, 1, DimVel> Fx_zero_vel{ Eigen::Matrix<double, 1, DimVel>::Zero() };
    Eigen::Matrix<double, 1, DimCtrl> Fx_zero_ctrl{ Eigen::Matrix<double, 1, DimCtrl>::Zero() };
    Eigen::Matrix<double, 1, DimParams> Fx_zero_params{ Eigen::Matrix<double, 1, DimParams>::Zero() };

    OptJacobian<1, DimVel> Ffy_H_xdot{ init_optional_jacobian(Ffy_zero_vel, Hxdot) };
    OptJacobian<1, 1> Ffy_H_delta{ init_optional_jacobian(Ffy_zero_delta, Hxdot) };
    OptJacobian<1, DimParams> Ffy_H_params{ init_optional_jacobian(Ffy_zero_params, Hxdot) };

    OptJacobian<1, DimVel> Fry_H_xdot{ init_optional_jacobian(Fry_zero_vel, Hxdot) };
    OptJacobian<1, 1> Fry_H_delta{ init_optional_jacobian(Fry_zero_delta, Hxdot) };
    OptJacobian<1, DimParams> Fry_H_params{ init_optional_jacobian(Fry_zero_params, Hxdot) };

    OptJacobian<1, DimVel> tauTV_H_xdot{ init_optional_jacobian(tauTV_zero_vel, Hxdot) };
    OptJacobian<1, 1> tauTV_H_delta{ init_optional_jacobian(tauTV_zero_delta, Hxdot) };
    OptJacobian<1, DimParams> tauTV_H_params{ init_optional_jacobian(tauTV_zero_params, Hxdot) };

    OptJacobian<1, DimVel> Fx_H_xdot{ init_optional_jacobian(Fx_zero_vel, Hxdot) };
    OptJacobian<1, DimCtrl> Fx_H_ctrl{ init_optional_jacobian(Fx_zero_ctrl, Hxdot) };
    OptJacobian<1, DimParams> Fx_H_params{ init_optional_jacobian(Fx_zero_params, Hxdot) };

    const double& vx{ xdot[0] };
    const double& vy{ xdot[1] };
    const double& r{ xdot[2] };

    const double& delta{ u[indices::delta] };

    const double& Iz{ params[indices::Iz] };
    const double& m{ static_params[indices::mass] };
    const double& lR{ static_params[indices::length_rear] };
    const double& lF{ static_params[indices::length_front] };

    const double Ffy{ lateral_force(xdot, delta, params, 'F', static_params,  // no-lint
                                    Ffy_H_xdot, Ffy_H_delta, Ffy_H_params)[0] };
    const double Fry{ lateral_force(xdot, delta, params, 'R', static_params,  // no-lint
                                    Fry_H_xdot, Fry_H_delta, Fry_H_params)[0] };
    const double tau_TV{ torque_vectoring_controller(delta, xdot, params, static_params,  // no-lint
                                                     tauTV_H_delta, tauTV_H_xdot, tauTV_H_params)[0] };

    const double Fx{ longitudinal_force(xdot, u, params, Fx_H_xdot, Fx_H_ctrl, Fx_H_params)[0] };

    const double vd_x{ (Fx - Ffy * std::sin(delta) + m * vy * r) / m };
    const double vd_y{ (Fry + Ffy * std::cos(delta) - m * vx * r) / m };
    const double rd{ (Ffy * lF * std::cos(delta) - Fry * lR + tau_TV) / Iz };

    if (Hxdot or Hctrl or Hparams)
    {
      const double dvdx_Ffy{ -std::sin(delta) / m };
      const double dvdy_Ffy{ std::cos(delta) / m };
      const double drd_Ffy{ (lF * std::cos(delta)) / Iz };

      const double dvdx_Fry{ 0.0 };
      const double dvdy_Fry{ 1.0 / m };
      const double drd_Fry{ -lR / Iz };

      const double drd_tauTv{ 1.0 / Iz };
      const double dvdx_Fx{ 1.0 / m };

      Eigen::Matrix<double, 3, 3> d_xdot{ Eigen::Matrix<double, 3, 3>::Zero() };
      d_xdot << 0, r, vy,  // no-lint
          -r, 0, -vx,      // no-lint
          0, 0, 0;

      const Eigen::Matrix<double, 3, 1> res_H_Ffy({ dvdx_Ffy, dvdy_Ffy, drd_Ffy });
      const Eigen::Matrix<double, 3, 1> res_H_Fry({ dvdx_Fry, dvdy_Fry, drd_Fry });
      const Eigen::Matrix<double, 3, 1> res_H_tauTV({ 0.0, 0.0, drd_tauTv });
      const Eigen::Matrix<double, 3, 1> res_H_Fx({ dvdx_Fx, 0.0, 0.0 });

      *Hxdot = res_H_Ffy * (*Ffy_H_xdot)        // no-lint
               + res_H_Fry * (*Fry_H_xdot)      // no-lint
               + res_H_tauTV * (*tauTV_H_xdot)  // no-lint
               + res_H_Fx * (*Fx_H_xdot)        // no-lint
               + d_xdot;
      if (Hctrl)
      {
        const double dvdx_delta{ -(Ffy * cos(delta)) / m };
        const double dvdy_delta{ -(Ffy * sin(delta)) / m };
        const double drd_delta{ -(Ffy * lF * sin(delta)) / Iz };

        const Eigen::Matrix<double, 3, 1> res_H_delta({ dvdx_delta, dvdy_delta, drd_delta });

        *Hctrl = res_H_Fx * (*Fx_H_ctrl);
        (*Hctrl).col(indices::delta) = res_H_delta +                 // no-lint
                                       res_H_Ffy * (*Ffy_H_delta) +  // no-lint
                                       res_H_Fry * (*Fry_H_delta) +  // no-lint
                                       res_H_tauTV * (*tauTV_H_delta);
      }
      if (Hparams)
      {
        const Eigen::Matrix<double, 3, 1> dres_Iz{ { 0.0, 0.0,
                                                     -(tau_TV - Fry * lR + Ffy * lF * std::cos(delta)) / (Iz * Iz) } };

        // const Eigen::Matrix<double, 3, 1> dres_m{ { (r * vy) / m - (Fx - Ffy * sin(delta) + m * r * vy) / (m * m),
        //                                             -(Fry + Ffy * std::cos(delta) - m * r * vx) / (m * m) - (r * vx)
        //                                             / m, 0.0 } };

        // const Eigen::Matrix<double, 3, 1> dres_lR{ { 0.0, 0.0, -Fry / Iz } };
        // const Eigen::Matrix<double, 3, 1> dres_lF{ { 0.0, 0.0, (Ffy * std::cos(delta)) / Iz } };

        Eigen::Matrix<double, 3, DimParams> dres_params{ Eigen::Matrix<double, 3, DimParams>::Zero() };

        dres_params.col(indices::Iz) = dres_Iz;

        *Hparams = res_H_Ffy * (*Ffy_H_params) +      // no-lint
                   res_H_Fry * (*Fry_H_params) +      // no-lint
                   res_H_tauTV * (*tauTV_H_params) +  // no-lint
                   res_H_Fx * (*Fx_H_params) +        // no-lint
                   dres_params;
      }
    }

    return { vd_x, vd_y, rd };
  }
};

struct dynamic_bicycle_sysid_factors_t
  : public gtsam::NoiseModelFactorN<bike_dynamics_t::Velocity, bike_dynamics_t::Velocity, bike_dynamics_t::Params>
{
  static constexpr Eigen::Index DimParams{ bike_dynamics_t::DimParams };

  using Base = gtsam::NoiseModelFactorN<bike_dynamics_t::Velocity, bike_dynamics_t::Velocity, bike_dynamics_t::Params>;
  using Velocity = bike_dynamics_t::Velocity;
  using Acceleration = bike_dynamics_t::Acceleration;
  using Control = bike_dynamics_t::Control;
  using StaticParams = bike_dynamics_t::StaticParams;
  using Params = bike_dynamics_t::Params;
  // using DimParams = bike_dynamics_t::DimParams;
  using OptDeriv = boost::optional<Eigen::MatrixXd&>;

  using EulerIntegrator = prx::fg::euler_integration_factor_t<Velocity, Acceleration, double>;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

  template <Eigen::Index DimOut, Eigen::Index DimIn>
  using OptJacobian = gtsam::OptionalJacobian<DimOut, DimIn>;

  dynamic_bicycle_sysid_factors_t(const gtsam::Key key_xdot1, const gtsam::Key key_xdot0, const gtsam::Key key_params,
                                  const NoiseModel& cost_model, const Control u, const double dt,
                                  const StaticParams& static_params)
    : Base(cost_model, key_xdot0, key_xdot1, key_params), _u(u), _dt(dt), _static_params(static_params)
  {
  }

  // X1 = X0 + f(x,u) dt
  virtual Eigen::VectorXd evaluateError(const Velocity& x1, const Velocity& x0, const Params& params,
                                        OptDeriv H1 = boost::none, OptDeriv H0 = boost::none,
                                        OptDeriv Hparams = boost::none) const override
  {
    Eigen::Matrix<double, 3, 3> zero_x0dd{ Eigen::Matrix<double, 3, 3>::Zero() };
    Eigen::Matrix<double, 3, DimParams> par_zero{ Eigen::Matrix<double, 3, DimParams>::Zero() };
    Eigen::MatrixXd zero_x1p{ Eigen::Matrix<double, 3, 3>::Zero() };
    Eigen::MatrixXd x1p_zero{ Eigen::Matrix<double, 3, 3>::Zero() };

    OptJacobian<3, 3> x0dd_H_x0{ init_optional_jacobian(zero_x0dd, H1, H0, Hparams) };
    OptJacobian<3, DimParams> x0dd_H_params{ init_optional_jacobian(par_zero, H1, H0, Hparams) };
    OptDeriv x1p_H_x0{ check_optional(zero_x1p, H1, H0, Hparams) };
    OptDeriv x1p_H_x0dd{ check_optional(x1p_zero, H1, H0, Hparams) };

    const Acceleration x0dd{ bike_dynamics_t::acceleration(x0, _u, params, _static_params, x0dd_H_x0, boost::none,
                                                           x0dd_H_params) };
    const Velocity x1p{ EulerIntegrator::integrate(x0, x0dd, _dt, x1p_H_x0, x1p_H_x0dd) };

    const Velocity error{ x1p - x1 };
    if (H1)
    {
      *H1 = -1.0 * Eigen::Matrix<double, 3, 3>::Identity();
    }
    if (H0)
    {
      //  err_H_x1p = I ;
      *H0 = (*x1p_H_x0) + (*x1p_H_x0dd) * (*x0dd_H_x0);
    }
    if (Hparams)
    {
      //  err_H_x1p = I ;
      *Hparams = (*x1p_H_x0dd) * (*x0dd_H_params);
    }

    return error;
  }

  const StaticParams _static_params;
  const Control _u;
  const double _dt;
};
}  // namespace fg
}  // namespace prx