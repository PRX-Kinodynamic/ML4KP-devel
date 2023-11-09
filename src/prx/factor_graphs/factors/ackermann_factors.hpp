#pragma once
#include <array>
#include <numeric>
#include <gtsam/config.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "prx/utilities/math/math_functions.hpp"
#include "prx/utilities/math/first_order_derivative.hpp"
// #include "prx/factor_graphs/utilities/prx_symbols.hpp"
#include "prx/simulation/plants/types/linear_time_variant.hpp"

#include "prx/factor_graphs/factors/noise_model_factor.hpp"
#include "prx/factor_graphs/utilities/constants.hpp"
#include "prx/factor_graphs/utilities/symbols_factory.hpp"

namespace prx
{
namespace fg
{

namespace ackermann
{
// Q = (x, y, \theta) --> position, velocity, steering angle
// Qdot = (\dot{x}, \dot{y}, \dot{\theta}) --> velocities
// Qdotdot = (\ddot{x}, \ddot{y}, \ddot{\theta}) --> acelerations
//
const Eigen::Index DimQ{ 3 };
const Eigen::Index DimQdot{ 3 };
const Eigen::Index DimQdotdot{ 3 };
const Eigen::Index DimModelParams{ 1 };
const Eigen::Index DimEnvironmentParams{ 1 };
const Eigen::Index DimForce{ 1 };
const Eigen::Index DimU{ 2 };

const Eigen::Index DimQz{ 3 };

using Q = Eigen::Vector<double, DimQ>;
using Qdot = Eigen::Vector<double, DimQdot>;
using Qdotdot = Eigen::Vector<double, DimQdotdot>;
using ModelParams = Eigen::Vector<double, DimModelParams>;
using EnvironmentParams = Eigen::Vector<double, DimEnvironmentParams>;
using Force = Eigen::Vector<double, DimForce>;

using U = Eigen::Vector<double, DimU>;

// clang-format off
inline double& steering(U& u) { return u[0]; };
inline double& velocity(U& u) { return u[1]; };

// double steering(const U& u) { return u[0]; };
// double velocity(const U& u) { return u[1]; };
// clang-format on

using Duration = Eigen::Vector<double, 1>;

using Qz = Eigen::Vector<double, DimQz>;

// Factor <- U_real, U_desired
// Error on U
class q_prop_factor_t : public gtsam::NoiseModelFactor3<ackermann::Q, ackermann::Q, ackermann::Qdot>
{
  using Q = ackermann::Q;
  using Qdot = ackermann::Qdot;

  using Base = gtsam::NoiseModelFactor3<Q, Q, Qdot>;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

  using Partial_Q0 = std::function<Q(const Q&)>;
  using Partial_Q1 = std::function<Q(const Q&)>;
  using Partial_Qdot = std::function<Q(const Qdot&)>;

public:
  q_prop_factor_t(const gtsam::Key key_q0, const gtsam::Key key_q1, const gtsam::Key key_qdot,
                  const NoiseModel& cost_model, const double h = prx::simulation_step)
    : Base(cost_model, key_q0, key_q1, key_qdot), derivative_q0(h), derivative_q1(h), derivative_qdot(h)
  {
  }

  virtual Eigen::VectorXd evaluateError(const Q& q0, const Q& q1, const Qdot& qdot,            // no-lint
                                        boost::optional<Eigen::MatrixXd&> H_q0 = boost::none,  // no-lint
                                        boost::optional<Eigen::MatrixXd&> H_q1 = boost::none,  // no-lint
                                        boost::optional<Eigen::MatrixXd&> H_qdot = boost::none) const override
  {
    if (H_q0)
    {
      derivative_q0._model = [&](const Q& q0_) { return compute_error(q0_, q1, qdot); };
      *H_q0 = derivative_q0(q0);
    }
    if (H_q1)
    {
      derivative_q1._model = [&](const Q& q1_) { return compute_error(q0, q1_, qdot); };
      *H_q1 = derivative_q1(q1);
    }
    if (H_qdot)
    {
      derivative_qdot._model = [&](const Qdot& qdot_) { return compute_error(q0, q1, qdot_); };
      *H_qdot = derivative_qdot(qdot);
    }

    return compute_error(q0, q1, qdot);
  }

  Q compute_error(const Q& q_0, const Q& q_1, const Qdot& qdot) const
  {
    Q q1_expected{ q_0 + qdot * prx::simulation_step };
    q1_expected[2] = norm_angle_pi(q1_expected[2]);
    Q error{ q_1 - q1_expected };
    error[2] = prx::fg::utilities::angle_diff(q_1[2], q1_expected[2]);
    return error;
  }

  // PRX_FACTOR_OVERLOAD_PRINT("ackermann_q_qdot_t");
  virtual void print(const std::string& s = "ackermann_q_qdot_t",
                     const gtsam::KeyFormatter& formatter = gtsam::DefaultKeyFormatter) const override
  {
    std::cout << "ackermann_q_qdot_t";

    for (gtsam::Key key : keys_)
      std::cout << " " << prx::symbol_factory_t::formatter(key);
    std::cout << " ";
  }

private:
  Partial_Q0 partial_q0;
  Partial_Q1 partial_q1;
  Partial_Qdot partial_qdot;

  mutable prx::math::first_order_derivative_t<Partial_Q0, Q, 4> derivative_q0;
  mutable prx::math::first_order_derivative_t<Partial_Q1, Q, 4> derivative_q1;
  mutable prx::math::first_order_derivative_t<Partial_Qdot, Qdot, 4> derivative_qdot;
};

class p_ctrl_factor_t : public gtsam::NoiseModelFactor2<U, U>
{
  using Gains = Eigen::DiagonalMatrix<double, DimU>;

  using Base = gtsam::NoiseModelFactor2<U, U>;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

  using Partial_U = std::function<U(const U&)>;

public:
  p_ctrl_factor_t(const gtsam::Key key_ur, const gtsam::Key key_ud, const NoiseModel& cost_model, const double ks,
                  const double kv, const double h = prx::simulation_step)
    : Base(cost_model, key_ur, key_ud), _derivative_ur(h), _derivative_ud(h), _gains(ks, kv)
  {
  }

  virtual Eigen::VectorXd evaluateError(const U& ur, const U& ud,                              // no-lint
                                        boost::optional<Eigen::MatrixXd&> H_ur = boost::none,  // no-lint
                                        boost::optional<Eigen::MatrixXd&> H_ud = boost::none) const override
  {
    if (H_ur)
    {
      _derivative_ur._model = [&](const U& ur_) { return compute_error(ur_, ud); };
      *H_ur = _derivative_ur(ur);
    }
    if (H_ud)
    {
      _derivative_ud._model = [&](const U& ud_) { return compute_error(ur, ud_); };
      *H_ud = _derivative_ur(ud);
    }

    return compute_error(ur, ud);
  }

  U compute_error(const U& ur, const U& ud) const
  {
    const U u_next{ ur + _gains * (ud - ur) };
    const U error{ u_next - ur };
    return error;
  }
  // PRX_FACTOR_OVERLOAD_PRINT("p_ctrl_factor_t");
  // virtual void print(const std::string& s = "ackermann_q_qdot_u_t",
  //                    const gtsam::KeyFormatter& formatter = gtsam::DefaultKeyFormatter) const override
  // {
  //   std::cout << "ackermann_q_qdot_u_t";

  //   for (gtsam::Key key : keys_)
  //     std::cout << " " << prx::symbol_factory_t::formatter(key);
  //   std::cout << " ";
  // }

private:
  const Gains _gains;  // velocity gain

  Partial_U _partial_ur;
  Partial_U _partial_ud;

  mutable prx::math::first_order_derivative_t<Partial_U, U, 4> _derivative_ur;
  mutable prx::math::first_order_derivative_t<Partial_U, U, 4> _derivative_ud;
};

// Factor <- Q, Qdot, Qdotdot
// Error on Qdot
class ackermann_q_qdot_qdotdot_t : public gtsam::NoiseModelFactor3<ackermann::Qdot, ackermann::Qdotdot, ackermann::Q>
{
  using Q = ackermann::Q;
  using Qdot = ackermann::Qdot;
  using Qdotdot = ackermann::Qdotdot;

  using Base = gtsam::NoiseModelFactor3<Qdot, Qdotdot, Q>;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

  using Partial_Qdot = std::function<Qdot(const Qdot&)>;
  using Partial_Qdotdot = std::function<Qdot(const Qdotdot&)>;
  using Partial_Q = std::function<Qdot(const Q&)>;

public:
  ackermann_q_qdot_qdotdot_t(const gtsam::Key key_qdot, const gtsam::Key key_qdotdot, const gtsam::Key key_q,
                             const NoiseModel& cost_model, const double length, const double h = prx::simulation_step)
    : Base(cost_model, key_qdot, key_qdotdot, key_q)
    , _L(length)
    , derivative_qdot(h)
    , derivative_qdotdot(h)
    , derivative_q(h)
  {
    prx_assert(prx::simulation_step > 0, "simulation_step not set");
  }

  virtual Eigen::VectorXd evaluateError(const Qdot& qdot, const Qdotdot& qdotdot, const Q& q,       // no-lint
                                        boost::optional<Eigen::MatrixXd&> H_qdot = boost::none,     // no-lint
                                        boost::optional<Eigen::MatrixXd&> H_qdotdot = boost::none,  // no-lint
                                        boost::optional<Eigen::MatrixXd&> H_q = boost::none) const override
  {
    if (H_qdot)
    {
      derivative_qdot._model = [&](const Qdot& qdot_) { return compute_error(qdot_, qdotdot, q); };
      *H_qdot = derivative_qdot(qdot);
    }
    if (H_qdotdot)
    {
      derivative_qdotdot._model = [&](const Qdotdot& qdotdot_) { return compute_error(qdot, qdotdot_, q); };
      *H_qdotdot = derivative_qdotdot(qdotdot);
    }
    if (H_q)
    {
      derivative_q._model = [&](const Q& q_) { return compute_error(qdot, qdotdot, q_); };
      *H_q = derivative_q(q);
    }

    return compute_error(qdot, qdotdot, q);
  }

  Qdot compute_error(const Qdot& qdot, const Qdotdot& qdotdot, const Q& q) const
  {
    const double theta{ q[2] };  // theta
    const double v{ q[3] };      // velocity
    const double phi{ q[4] };    // steering

    Qdot qdot_c1{ Qdot::Zero() };  // current at 1
    Qdot qdot_p1{ Qdot::Zero() };  // expected at 1

    qdot_c1 = qdot + qdotdot * simulation_step;

    qdot_p1[0] = v * std::cos(theta);
    qdot_p1[1] = v * std::sin(theta);
    qdot_p1[2] = (v / _L) * std::tan(phi);
    return qdot_p1 - qdot_c1;
  }

  virtual void print(const std::string& s = "ackermann_q_qdot_qdotdot_t",
                     const gtsam::KeyFormatter& formatter = gtsam::DefaultKeyFormatter) const override
  {
    std::cout << "ackermann_q_qdot_qdotdot_t";

    for (gtsam::Key key : keys_)
      std::cout << " " << prx::symbol_factory_t::formatter(key);
    std::cout << " ";
  }

private:
  const double _L;

  Partial_Qdot partial_qdot;
  Partial_Qdotdot partial_qdotdot;
  Partial_Q partial_q;

  mutable prx::math::first_order_derivative_t<Partial_Qdot, Qdot, 4> derivative_qdot;
  mutable prx::math::first_order_derivative_t<Partial_Qdotdot, Qdotdot, 4> derivative_qdotdot;
  mutable prx::math::first_order_derivative_t<Partial_Q, Q, 4> derivative_q;
};

// Factor <- Q, Qdotdot, F
// Error on Qdotdot
class ackermann_qdotdot_force_q_t
  : public gtsam::NoiseModelFactor5<ackermann::Qdotdot, ackermann::Force, ackermann::Q, ackermann::ModelParams,
                                    ackermann::EnvironmentParams>
{
  using Q = ackermann::Q;
  using Qdot = ackermann::Qdot;
  using Qdotdot = ackermann::Qdotdot;
  using Force = ackermann::Force;
  using ModelParams = ackermann::ModelParams;
  using EnvironmentParams = ackermann::EnvironmentParams;

  using Base = gtsam::NoiseModelFactor5<Qdotdot, Force, Q, ModelParams, EnvironmentParams>;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

  using Partial_Qdotdot = std::function<Qdotdot(const Qdotdot&)>;
  using Partial_Force = std::function<Qdotdot(const Force&)>;
  using Partial_Q = std::function<Qdotdot(const Q&)>;
  using Partial_ModelParams = std::function<Qdotdot(const ModelParams&)>;
  using Partial_EnvironmentParams = std::function<Qdotdot(const EnvironmentParams&)>;

public:
  ackermann_qdotdot_force_q_t(const gtsam::Key key_qdotdot, const gtsam::Key key_force, const gtsam::Key key_q,
                              const gtsam::Key key_model_params, const gtsam::Key key_environment_params,
                              const NoiseModel& cost_model, const double wheel_distance, const double mass,
                              const double h = prx::simulation_step)
    : Base(cost_model, key_qdotdot, key_force, key_q, key_model_params, key_environment_params)
    , derivative_qdotdot(h)
    , derivative_force(h)
    , derivative_q(h)
    , derivative_model_params(h)
    , derivative_environment_params(h)
    , _wheel_distance(wheel_distance)
    , _mass(mass)
  {
  }

  virtual Eigen::VectorXd
  evaluateError(const Qdotdot& qdotdot, const Force& force, const Q& q, const ModelParams& model_params,
                const EnvironmentParams& environment_params,                     // no-lint
                boost::optional<Eigen::MatrixXd&> H_qdotdot = boost::none,       // no-lint
                boost::optional<Eigen::MatrixXd&> H_force = boost::none,         // no-lint
                boost::optional<Eigen::MatrixXd&> H_q = boost::none,             // no-lint
                boost::optional<Eigen::MatrixXd&> H_model_params = boost::none,  // no-lint
                boost::optional<Eigen::MatrixXd&> H_environemnt_params = boost::none) const override
  {
    if (H_qdotdot)
    {
      derivative_qdotdot._model = [&](const Qdotdot& qdotdot_) {
        return compute_error(qdotdot_, force, q, model_params, environment_params);
      };
      *H_qdotdot = derivative_qdotdot(qdotdot);
    }
    if (H_force)
    {
      derivative_force._model = [&](const Force& force_) {
        return compute_error(qdotdot, force_, q, model_params, environment_params);
      };
      *H_force = derivative_force(force);
    }
    if (H_q)
    {
      derivative_q._model = [&](const Q& q_) {
        return compute_error(qdotdot, force, q_, model_params, environment_params);
      };
      *H_q = derivative_q(q);
    }
    if (H_model_params)
    {
      derivative_model_params._model = [&](const ModelParams& model_params_) {
        return compute_error(qdotdot, force, q, model_params_, environment_params);
      };
      *H_model_params = derivative_model_params(model_params);
    }
    if (H_environemnt_params)
    {
      derivative_environment_params._model = [&](const EnvironmentParams& env_params_) {
        return compute_error(qdotdot, force, q, model_params, env_params_);
      };
      *H_environemnt_params = derivative_environment_params(environment_params);
    }

    return compute_error(qdotdot, force, q, model_params, environment_params);
  };

  Qdotdot compute_error(const Qdotdot& qdotdot, const Force& force, const Q& q, const ModelParams& model_params,
                        const EnvironmentParams environment_params) const
  {
    const double I{ model_params[0] };
    const double mu{ environment_params[0] };
    // const double mu{ std::exp(-environment_params[0] * 0.1) };

    const double f{ force[0] };

    const double theta{ q[2] };
    const double r_icc{ _wheel_distance / std::cos(theta) };

    Qdotdot qdotdot_p{ Qdotdot::Zero() };

    qdotdot_p[0] = mu * f * std::cos(theta) / _mass;
    qdotdot_p[1] = mu * f * std::sin(theta) / _mass;
    qdotdot_p[2] = (r_icc / I) * f;

    // PRX_DEBUG_VAR_1("----------");
    // PRX_DEBUG_VAR_1(qdotdot.transpose());
    // PRX_DEBUG_VAR_1(force.transpose());
    // PRX_DEBUG_VAR_1(q.transpose());
    // PRX_DEBUG_VAR_1(model_params.transpose());
    // PRX_DEBUG_VAR_1(environment_params.transpose());
    // PRX_DEBUG_VAR_1(mu);
    // PRX_DEBUG_VAR_1(qdotdot_p.transpose());
    // PRX_DEBUG_VAR_1((qdotdot - qdotdot_p).transpose());
    return qdotdot - qdotdot_p;
  }

  virtual void print(const std::string& s = "ackermann_qdotdot_force_q_t",
                     const gtsam::KeyFormatter& formatter = gtsam::DefaultKeyFormatter) const override
  {
    std::cout << "ackermann_qdotdot_force_q_t";

    for (gtsam::Key key : keys_)
      std::cout << " " << prx::symbol_factory_t::formatter(key);
    std::cout << " ";
  }

private:
  const double _wheel_distance;
  const double _mass;

  Partial_Qdotdot partial_qdotdot;
  Partial_Force partial_force;
  Partial_Q partial_q;
  Partial_ModelParams partial_model_params;
  Partial_EnvironmentParams partial_environment_params;

  mutable prx::math::first_order_derivative_t<Partial_Qdotdot, Qdotdot, 4> derivative_qdotdot;
  mutable prx::math::first_order_derivative_t<Partial_Force, Force, 4> derivative_force;
  mutable prx::math::first_order_derivative_t<Partial_Q, Q, 4> derivative_q;
  mutable prx::math::first_order_derivative_t<Partial_ModelParams, ModelParams, 4> derivative_model_params;
  mutable prx::math::first_order_derivative_t<Partial_EnvironmentParams, EnvironmentParams, 4>
      derivative_environment_params;
};

// Factor <- Q observed aka (x,y,\theta)
// Error is Q - (x,y,\theta,0,0)
class ackermann_q_observation_t : public noise_model_1p1_factor_t<ackermann::DimQz, ackermann::DimQ>
{
  using Q = ackermann::Q;
  using Qz = ackermann::Qz;

  using Base = noise_model_1p1_factor_t<ackermann::DimQz, ackermann::DimQ>;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

public:
  ackermann_q_observation_t(const Qz observation, const gtsam::Key key_q, const NoiseModel& cost_model)
    : Base(key_q, cost_model), _observation(observation)
  {
  }

  Qz compute_error(const Q& q) const override
  {
    return q.head(3) - _observation;
  }

  virtual void print(const std::string& s = "ackermann_q_observation_t",
                     const gtsam::KeyFormatter& formatter = gtsam::DefaultKeyFormatter) const override
  {
    std::cout << "ackermann_q_observation_t";

    for (gtsam::Key key : keys_)
      std::cout << " " << prx::symbol_factory_t::formatter(key);
    std::cout << " ";
  }

private:
  const Qz _observation;
};

// Factor <- Q observed aka (x,y,\theta)
// Error is Q - (x,y,\theta,0,0)
// class ackermann_q_observation_t : public gtsam::NoiseModelFactor2<ackermann::Qz, ackermann::Q>
// {
// };
}  // namespace ackermann

}  // namespace fg
}  // namespace prx