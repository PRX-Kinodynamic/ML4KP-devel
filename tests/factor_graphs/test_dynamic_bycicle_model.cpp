#define BOOST_AUTO_TEST_MAIN quadratic_cost_factor_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/defs.hpp"
#include "prx/factor_graphs/factors/dynamic_bicycle_factors.hpp"
#include "prx/utilities/math/first_order_derivative.hpp"

#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/geometry/SOn.h>

using Params = prx::fg::bike_dynamics_t::Params;
using Velocity = prx::fg::bike_dynamics_t::Velocity;
using Control = prx::fg::bike_dynamics_t::Control;
using StaticParams = prx::fg::bike_dynamics_t::StaticParams;
using Vector1D = prx::fg::bike_dynamics_t::Vector1D;

using Factors = prx::fg::bike_dynamics_t;

BOOST_AUTO_TEST_CASE(dynamic_bicycle_factors_slip_angle_derivs)
{
  using namespace std::placeholders;
  using SlipVelocityPartial = std::function<Vector1D(const Velocity&)>;
  using SlipDeltaPartial = std::function<Vector1D(const double&)>;
  StaticParams static_params{ { 1, 1, 1 } };
  const double delta_cte{ 0.5 };
  const Velocity vel_cte{ { 1.0, 2.0, 3.0 } };

  SlipVelocityPartial slip_angle_vel_f =
      std::bind(&Factors::slip_angle, _1, delta_cte, 'F', static_params, boost::none, boost::none);
  SlipVelocityPartial slip_angle_vel_r =
      std::bind(&Factors::slip_angle, _1, delta_cte, 'R', static_params, boost::none, boost::none);

  SlipDeltaPartial slip_angle_delta_f =
      std::bind(&Factors::slip_angle, vel_cte, _1, 'F', static_params, boost::none, boost::none);
  SlipDeltaPartial slip_angle_delta_r =
      std::bind(&Factors::slip_angle, vel_cte, _1, 'R', static_params, boost::none, boost::none);

  using DerivativeVelF = prx::math::first_order_derivative_t<decltype(slip_angle_vel_f), Velocity, 4, -1>;
  using DerivativeVelR = prx::math::first_order_derivative_t<decltype(slip_angle_vel_r), Velocity, 4, -1>;

  using DerivativeDeltaF = prx::math::first_order_derivative_t<decltype(slip_angle_delta_f), double, 4, -1>;
  using DerivativeDeltaR = prx::math::first_order_derivative_t<decltype(slip_angle_delta_r), double, 4, -1>;

  DerivativeVelF dxdot_f(slip_angle_vel_f, 0.01, 3, 1);
  DerivativeVelR dxdot_r(slip_angle_vel_r, 0.01, 3, 1);

  DerivativeDeltaF ddelta_f(slip_angle_delta_f, 0.01, 1, 1);
  DerivativeDeltaR ddelta_r(slip_angle_delta_r, 0.01, 1, 1);

  Velocity vel(1.0, 2.0, 3.0);
  Eigen::MatrixXd Hxdot_f_computed, Hxdot_r_computed;
  Eigen::MatrixXd Hdelta_f_computed, Hdelta_r_computed;

  auto Hxdot_f_expected = dxdot_f(vel);
  auto Hxdot_r_expected = dxdot_r(vel);

  auto Hdelta_f_expected = ddelta_f(delta_cte);
  auto Hdelta_r_expected = ddelta_r(delta_cte);

  Factors::slip_angle(vel, delta_cte, 'F', static_params, Hxdot_f_computed, Hdelta_f_computed);
  Factors::slip_angle(vel, delta_cte, 'R', static_params, Hxdot_r_computed, Hdelta_r_computed);

  // PRX_DBG_VARS(Hxdot_f_expected);
  // PRX_DBG_VARS(Hxdot_f_computed);
  // DerivativeDelta ddelta(0.01);
  BOOST_REQUIRE_MESSAGE((Hxdot_f_expected - Hxdot_f_computed).isZero(1e-5),
                        EXPECTED_GOT(Hxdot_f_expected, Hxdot_f_computed));
  BOOST_REQUIRE_MESSAGE((Hxdot_r_expected - Hxdot_r_computed).isZero(1e-5),
                        EXPECTED_GOT(Hxdot_r_expected, Hxdot_r_computed));

  BOOST_REQUIRE_MESSAGE((Hdelta_f_expected - Hdelta_f_computed).isZero(1e-5),
                        EXPECTED_GOT(Hdelta_f_expected, Hdelta_f_computed));
  BOOST_REQUIRE_MESSAGE((Hdelta_r_expected - Hdelta_r_computed).isZero(1e-5),
                        EXPECTED_GOT(Hdelta_r_expected, Hdelta_r_computed));
}

BOOST_AUTO_TEST_CASE(dynamic_bicycle_factors_lateral_force_derivs)
{
  using namespace std::placeholders;
  using LateralForceVelocityPartial = std::function<Vector1D(const Velocity&)>;
  using LateralForceDeltaPartial = std::function<Vector1D(const double&)>;
  using LateralForceParamsPartial = std::function<Vector1D(const Params&)>;

  using DerivativeVel = prx::math::first_order_derivative_t<LateralForceVelocityPartial, Velocity, 4, -1>;
  using DerivativeDelta = prx::math::first_order_derivative_t<LateralForceDeltaPartial, double, 4, -1>;
  using DerivativeParams = prx::math::first_order_derivative_t<LateralForceParamsPartial, Params, 4, -1>;

  Params params{ Params::Ones() };
  StaticParams static_params{ { 1, 1, 1 } };
  const double delta_cte{ 0.5 };
  const Velocity vel_cte{ { 1.0, 2.0, 3.0 } };

  LateralForceVelocityPartial lateral_force_vel_f = std::bind(&Factors::lateral_force, _1, delta_cte, params, 'F',
                                                              static_params, boost::none, boost::none, boost::none);
  LateralForceVelocityPartial lateral_force_vel_r = std::bind(&Factors::lateral_force, _1, delta_cte, params, 'R',
                                                              static_params, boost::none, boost::none, boost::none);
  LateralForceDeltaPartial lateral_force_delta_f = std::bind(&Factors::lateral_force, vel_cte, _1, params, 'F',
                                                             static_params, boost::none, boost::none, boost::none);
  LateralForceDeltaPartial lateral_force_delta_r = std::bind(&Factors::lateral_force, vel_cte, _1, params, 'R',
                                                             static_params, boost::none, boost::none, boost::none);
  LateralForceParamsPartial lateral_force_params_f = std::bind(&Factors::lateral_force, vel_cte, delta_cte, _1, 'F',
                                                               static_params, boost::none, boost::none, boost::none);
  LateralForceParamsPartial lateral_force_params_r = std::bind(&Factors::lateral_force, vel_cte, delta_cte, _1, 'R',
                                                               static_params, boost::none, boost::none, boost::none);

  DerivativeVel dxdot_f(lateral_force_vel_f, 0.01, 3, 1);
  DerivativeVel dxdot_r(lateral_force_vel_r, 0.01, 3, 1);
  DerivativeDelta ddelta_f(lateral_force_delta_f, 0.01, 1, 1);
  DerivativeDelta ddelta_r(lateral_force_delta_r, 0.01, 1, 1);
  DerivativeParams dparams_f(lateral_force_params_f, 0.01, Factors::DimParams, 1);
  DerivativeParams dparams_r(lateral_force_params_r, 0.01, Factors::DimParams, 1);

  Eigen::MatrixXd Hxdot_f_computed, Hxdot_r_computed;
  Eigen::MatrixXd Hdelta_f_computed, Hdelta_r_computed;
  Eigen::MatrixXd Hparams_f_computed, Hparams_r_computed;

  auto Hxdot_f_expected = dxdot_f(vel_cte);
  auto Hxdot_r_expected = dxdot_r(vel_cte);
  auto Hdelta_f_expected = ddelta_f(delta_cte);
  auto Hdelta_r_expected = ddelta_r(delta_cte);
  auto Hparams_f_expected = dparams_f(params);
  auto Hparams_r_expected = dparams_r(params);

  Factors::lateral_force(vel_cte, delta_cte, params, 'F', static_params,  // no-lint
                         Hxdot_f_computed, Hdelta_f_computed, Hparams_f_computed);
  Factors::lateral_force(vel_cte, delta_cte, params, 'R', static_params,  // no-lint
                         Hxdot_r_computed, Hdelta_r_computed, Hparams_r_computed);

  // PRX_DBG_VARS(Hxdot_f_expected);
  // PRX_DBG_VARS(Hxdot_f_computed);
  // DerivativeDelta ddelta(0.01);
  BOOST_REQUIRE_MESSAGE((Hxdot_f_expected - Hxdot_f_computed).isZero(1e-5),
                        EXPECTED_GOT(Hxdot_f_expected, Hxdot_f_computed));
  BOOST_REQUIRE_MESSAGE((Hxdot_r_expected - Hxdot_r_computed).isZero(1e-5),
                        EXPECTED_GOT(Hxdot_r_expected, Hxdot_r_computed));

  BOOST_REQUIRE_MESSAGE((Hdelta_f_expected - Hdelta_f_computed).isZero(1e-5),
                        EXPECTED_GOT(Hdelta_f_expected, Hdelta_f_computed));
  BOOST_REQUIRE_MESSAGE((Hdelta_r_expected - Hdelta_r_computed).isZero(1e-5),
                        EXPECTED_GOT(Hdelta_r_expected, Hdelta_r_computed));

  BOOST_REQUIRE_MESSAGE((Hparams_f_expected - Hparams_f_computed).isZero(1e-5),
                        EXPECTED_GOT(Hparams_f_expected, Hparams_f_computed));
  BOOST_REQUIRE_MESSAGE((Hparams_r_expected - Hparams_r_computed).isZero(1e-5),
                        EXPECTED_GOT(Hparams_r_expected, Hparams_r_computed));
  // BOOST_REQUIRE_CLOSE(x0_error[0], x0_expected, 1e-3);
  // BOOST_REQUIRE_CLOSE(x1_error[0], x1_expected, 1e-3);
}

BOOST_AUTO_TEST_CASE(dynamic_bicycle_factors_longitudinal_force_derivs)
{
  using namespace std::placeholders;

  using LongitudinalForceVelocityPartial = std::function<Vector1D(const Velocity&)>;
  using LongitudinalForceControlPartial = std::function<Vector1D(const Control&)>;
  using LongitudinalForceParamsPartial = std::function<Vector1D(const Params&)>;

  using DerivativeVel = prx::math::first_order_derivative_t<LongitudinalForceVelocityPartial, Velocity, 4, -1>;
  using DerivativeCtrl = prx::math::first_order_derivative_t<LongitudinalForceControlPartial, Control, 4, -1>;
  using DerivativeParams = prx::math::first_order_derivative_t<LongitudinalForceParamsPartial, Params, 4, -1>;

  Params params{ Params::Ones() };
  const Control ctrl_cte{ { 0.5, 0.75 } };
  const Velocity vel_cte{ { 1.0, 2.0, 3.0 } };

  LongitudinalForceVelocityPartial lateral_force_vel =
      std::bind(&Factors::longitudinal_force, _1, ctrl_cte, params, boost::none, boost::none, boost::none);
  LongitudinalForceControlPartial longitudinal_force_delta =
      std::bind(&Factors::longitudinal_force, vel_cte, _1, params, boost::none, boost::none, boost::none);
  LongitudinalForceParamsPartial longitudinal_force_params =
      std::bind(&Factors::longitudinal_force, vel_cte, ctrl_cte, _1, boost::none, boost::none, boost::none);

  DerivativeVel dxdot(lateral_force_vel, 0.01, 3, 1);
  DerivativeCtrl dctrl(longitudinal_force_delta, 0.01, 2, 1);
  DerivativeParams dparams(longitudinal_force_params, 0.01, Factors::DimParams, 1);

  Eigen::MatrixXd Hxdot_computed;
  Eigen::MatrixXd Hctrl_computed;
  Eigen::MatrixXd Hparams_computed;

  auto Hxdot_expected = dxdot(vel_cte);
  auto Hctrl_expected = dctrl(ctrl_cte);
  auto Hparams_expected = dparams(params);

  Factors::longitudinal_force(vel_cte, ctrl_cte, params,  // no-lint
                              Hxdot_computed, Hctrl_computed, Hparams_computed);

  BOOST_REQUIRE_MESSAGE((Hxdot_expected - Hxdot_computed).isZero(1e-5),  // no-lint
                        EXPECTED_GOT(Hxdot_expected, Hxdot_computed));

  BOOST_REQUIRE_MESSAGE((Hctrl_expected - Hctrl_computed).isZero(1e-5),  // no-lint
                        EXPECTED_GOT(Hctrl_expected, Hctrl_computed));

  BOOST_REQUIRE_MESSAGE((Hparams_expected - Hparams_computed).isZero(1e-5),  // no-lint
                        EXPECTED_GOT(Hparams_expected, Hparams_computed));
}

BOOST_AUTO_TEST_CASE(dynamic_bicycle_factors_torque_vectoring_controller_derivs)
{
  using namespace std::placeholders;

  using TorqueVectoringControllerDeltaPartial = std::function<Vector1D(const double&)>;
  using TorqueVectoringControllerVelocityPartial = std::function<Vector1D(const Velocity&)>;
  using TorqueVectoringControllerParamsPartial = std::function<Vector1D(const Params&)>;

  using DerivativeDelta = prx::math::first_order_derivative_t<TorqueVectoringControllerDeltaPartial, double, 4, -1>;
  using DerivativeVelocity =
      prx::math::first_order_derivative_t<TorqueVectoringControllerVelocityPartial, Velocity, 4, -1>;
  using DerivativeParams = prx::math::first_order_derivative_t<TorqueVectoringControllerParamsPartial, Params, 4, -1>;

  StaticParams static_params{ StaticParams::Ones() };
  Params params{ Params::Ones() };
  const double delta_cte{ 0.5 };
  const Velocity vel_cte{ { 1.0, 2.0, 3.0 } };

  TorqueVectoringControllerDeltaPartial torque_vectoring_controller_delta = std::bind(
      &Factors::torque_vectoring_controller, _1, vel_cte, params, static_params, boost::none, boost::none, boost::none);
  TorqueVectoringControllerVelocityPartial torque_vectoring_controller_vel =
      std::bind(&Factors::torque_vectoring_controller, delta_cte, _1, params, static_params, boost::none, boost::none,
                boost::none);
  TorqueVectoringControllerParamsPartial torque_vectoring_controller_params =
      std::bind(&Factors::torque_vectoring_controller, delta_cte, vel_cte, _1, static_params, boost::none, boost::none,
                boost::none);

  DerivativeDelta ddelta(torque_vectoring_controller_delta, 0.01, 1, 1);
  DerivativeVelocity dvel(torque_vectoring_controller_vel, 0.01, 3, 1);
  DerivativeParams dparams(torque_vectoring_controller_params, 0.01, Factors::DimParams, 1);

  Eigen::MatrixXd Hdelta_computed;
  Eigen::MatrixXd Hvel_computed;
  Eigen::MatrixXd Hparams_computed;

  auto Hdelta_expected = ddelta(delta_cte);
  auto Hvel_expected = dvel(vel_cte);
  auto Hparams_expected = dparams(params);

  Factors::torque_vectoring_controller(delta_cte, vel_cte, params, static_params,  // no-lint
                                       Hdelta_computed, Hvel_computed, Hparams_computed);

  BOOST_REQUIRE_MESSAGE((Hdelta_expected - Hdelta_computed).isZero(1e-5),  // no-lint
                        EXPECTED_GOT(Hdelta_expected, Hdelta_computed));

  BOOST_REQUIRE_MESSAGE((Hvel_expected - Hvel_computed).isZero(1e-5),  // no-lint
                        EXPECTED_GOT(Hvel_expected, Hvel_computed));

  BOOST_REQUIRE_MESSAGE((Hparams_expected - Hparams_computed).isZero(1e-5),  // no-lint
                        EXPECTED_GOT(Hparams_expected, Hparams_computed));
}

BOOST_AUTO_TEST_CASE(dynamic_bicycle_factors_acceleration_derivs)
{
  using namespace std::placeholders;

  using AccelerationVelocityPartial = std::function<Eigen::Vector3d(const Velocity&)>;
  using AccelerationControlPartial = std::function<Eigen::Vector3d(const Control&)>;
  using AccelerationParamsPartial = std::function<Eigen::Vector3d(const Params&)>;

  using DerivativeVelocity = prx::math::first_order_derivative_t<AccelerationVelocityPartial, Velocity, 4, -1>;
  using DerivativeControl = prx::math::first_order_derivative_t<AccelerationControlPartial, Control, 4, -1>;
  using DerivativeParams = prx::math::first_order_derivative_t<AccelerationParamsPartial, Params, 4, -1>;

  StaticParams static_params{ StaticParams::Ones() };
  Params params{ Params::Ones() };
  const Control ctrl_cte{ { 0.5, 0.75 } };
  const Velocity vel_cte{ { 1.0, 2.0, 3.0 } };

  AccelerationVelocityPartial acceleration_vel =
      std::bind(&Factors::acceleration, _1, ctrl_cte, params, static_params, boost::none, boost::none, boost::none);
  AccelerationControlPartial acceleration_ctrl =
      std::bind(&Factors::acceleration, vel_cte, _1, params, static_params, boost::none, boost::none, boost::none);
  AccelerationParamsPartial acceleration_params =
      std::bind(&Factors::acceleration, vel_cte, ctrl_cte, _1, static_params, boost::none, boost::none, boost::none);

  DerivativeVelocity dvel(acceleration_vel, 0.01, 3, 3);
  DerivativeControl dctrl(acceleration_ctrl, 0.01, 2, 3);
  DerivativeParams dparams(acceleration_params, 0.01, Factors::DimParams, 3);

  Eigen::MatrixXd Hvel_computed;
  Eigen::MatrixXd Hctrl_computed;
  Eigen::MatrixXd Hparams_computed;

  auto Hvel_expected = dvel(vel_cte);
  auto Hctrl_expected = dctrl(ctrl_cte);
  auto Hparams_expected = dparams(params);

  Factors::acceleration(vel_cte, ctrl_cte, params, static_params,  // no-lint
                        Hvel_computed, Hctrl_computed, Hparams_computed);

  BOOST_REQUIRE_MESSAGE((Hvel_expected - Hvel_computed).isZero(1e-5),  // no-lint
                        EXPECTED_GOT(Hvel_expected, Hvel_computed));

  BOOST_REQUIRE_MESSAGE((Hctrl_expected - Hctrl_computed).isZero(1e-5),  // no-lint
                        EXPECTED_GOT(Hctrl_expected, Hctrl_computed));

  BOOST_REQUIRE_MESSAGE((Hparams_expected - Hparams_computed).isZero(1e-5),  // no-lint
                        EXPECTED_GOT(Hparams_expected, Hparams_computed));
}

BOOST_AUTO_TEST_CASE(dynamic_bicycle_factors_sysid_derivs)
{
  using namespace std::placeholders;
  using BikeSysIdFactor = prx::fg::dynamic_bicycle_sysid_factors_t;

  using SysIdFactorVelocityPartial = std::function<Eigen::Vector3d(const Velocity&)>;
  using SysIdFactorParamsPartial = std::function<Eigen::Vector3d(const Params&)>;

  using DerivativeVelocity = prx::math::first_order_derivative_t<SysIdFactorVelocityPartial, Velocity, 4, -1>;
  using DerivativeParams = prx::math::first_order_derivative_t<SysIdFactorParamsPartial, Params, 4, -1>;

  StaticParams static_params{ StaticParams::Ones() };
  Params params{ Params::Ones() };
  const double dt{ 0.1 };
  const Control ctrl_cte{ { 0.5, 0.75 } };
  const Velocity xdot0_cte{ { 1.0, 2.0, 3.0 } };
  const Velocity xdot1_cte{ xdot0_cte + Velocity::Ones() * dt };
  gtsam::noiseModel::Base::shared_ptr noise_model{ gtsam::noiseModel::Isotropic::Sigma(3, 1) };

  BikeSysIdFactor sysid_factor(0, 1, 2, noise_model, ctrl_cte, dt, static_params);

  SysIdFactorVelocityPartial sysid_xdot1 = std::bind(&BikeSysIdFactor::evaluateError, &sysid_factor, _1, xdot0_cte,
                                                     params, boost::none, boost::none, boost::none);
  SysIdFactorVelocityPartial sysid_xdot0 = std::bind(&BikeSysIdFactor::evaluateError, &sysid_factor, xdot1_cte, _1,
                                                     params, boost::none, boost::none, boost::none);
  SysIdFactorParamsPartial sysid_params = std::bind(&BikeSysIdFactor::evaluateError, &sysid_factor, xdot1_cte,
                                                    xdot0_cte, _1, boost::none, boost::none, boost::none);

  DerivativeVelocity dxdot1(sysid_xdot1, 0.01);
  DerivativeVelocity dxdot0(sysid_xdot0, 0.01);
  DerivativeParams dparams(sysid_params, 0.01);

  Eigen::MatrixXd Hxdot1_computed;
  Eigen::MatrixXd Hxdot0_computed;
  Eigen::MatrixXd Hparams_computed;

  auto Hxdot1_expected = dxdot1(xdot1_cte);
  auto Hxdot0_expected = dxdot0(xdot0_cte);
  auto Hparams_expected = dparams(params);

  sysid_factor.evaluateError(xdot1_cte, xdot0_cte, params,  // no-lint
                             Hxdot1_computed, Hxdot0_computed, Hparams_computed);

  BOOST_REQUIRE_MESSAGE((Hxdot1_expected - Hxdot1_computed).isZero(1e-5),  // no-lint
                        EXPECTED_GOT(Hxdot1_expected, Hxdot1_computed));

  BOOST_REQUIRE_MESSAGE((Hxdot0_expected - Hxdot0_computed).isZero(1e-5),  // no-lint
                        EXPECTED_GOT(Hxdot0_expected, Hxdot0_computed));

  BOOST_REQUIRE_MESSAGE((Hparams_expected - Hparams_computed).isZero(1e-5),  // no-lint
                        EXPECTED_GOT(Hparams_expected, Hparams_computed));
}