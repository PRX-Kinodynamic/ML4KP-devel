#pragma once
#include <array>
#include <numeric>
#include <gtsam/config.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "prx/utilities/math/math_functions.hpp"
// #include "prx/factor_graphs/utilities/prx_symbols.hpp"
#include "prx/factor_graphs/utilities/symbols_factory.hpp"
#include "prx/simulation/plants/types/linear_time_variant.hpp"

namespace prx
{
namespace fg
{

namespace ackermann
{
// Q = (x, y, \theta, V, \phi) --> position, velocity, steering angle
// Qdot = (\dot{x}, \dot{y}, \dot{\theta}) --> velocities
// Qdotdot = (\ddot{x}, \ddot{y}, \ddot{\theta}) --> acelerations
//
const Eigen::Index DimQ{ 5 };
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

using Qz = Eigen::Vector<double, DimQz>;
}  // namespace ackermann

// Factor <- Q, Qdot, U
// Error on Q
class ackermann_q_qdot_u_t : public gtsam::NoiseModelFactor4<ackermann::Q, ackermann::Q, ackermann::Qdot, ackermann::U>
{
  using Q = ackermann::Q;
  using Qdot = ackermann::Qdot;
  using U = ackermann::U;

  using Base = gtsam::NoiseModelFactor4<Q, Q, Qdot, U>;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

  using Partial_Q0 = std::function<Q(const Q&)>;
  using Partial_Q1 = std::function<Q(const Q&)>;
  using Partial_Qdot = std::function<Q(const Qdot&)>;
  using Partial_U = std::function<Q(const U&)>;

public:
  ackermann_q_qdot_u_t(const gtsam::Key key_q0, const gtsam::Key key_q1, const gtsam::Key key_qdot,
                       const gtsam::Key key_u, const NoiseModel& cost_model, const double h = prx::simulation_step)
    : Base(cost_model, key_q0, key_q1, key_qdot, key_u)
    , derivative_q0(h)
    , derivative_q1(h)
    , derivative_qdot(h)
    , derivative_u(h)
  {
  }

  virtual Eigen::VectorXd evaluateError(const Q& q0, const Q& q1, const Qdot& qdot, const U& u,  // no-lint
                                        boost::optional<Eigen::MatrixXd&> H_q0 = boost::none,    // no-lint
                                        boost::optional<Eigen::MatrixXd&> H_q1 = boost::none,    // no-lint
                                        boost::optional<Eigen::MatrixXd&> H_qdot = boost::none,  // no-lint
                                        boost::optional<Eigen::MatrixXd&> H_u = boost::none) const override
  {
    if (H_q0)
    {
      derivative_q0._model = [&](const Q& q0_) { return compute_error(q0_, q1, qdot, u); };
      *H_q0 = derivative_q0(q0);
    }
    if (H_q1)
    {
      derivative_q1._model = [&](const Q& q1_) { return compute_error(q0, q1_, qdot, u); };
      *H_q1 = derivative_q1(q1);
    }
    if (H_qdot)
    {
      derivative_qdot._model = [&](const Qdot& qdot_) { return compute_error(q0, q1, qdot_, u); };
      *H_qdot = derivative_qdot(qdot);
    }
    if (H_u)
    {
      derivative_u._model = [&](const U& u_) { return compute_error(q0, q1, qdot, u_); };
      *H_u = derivative_u(u);
    }

    return compute_error(q0, q1, qdot, u);
  }

  Q compute_error(const Q& q_0, const Q& q_1, const Qdot& qdot, const U& u) const
  {
    const double v_0{ q_0[3] };    // velocity at 0
    const double phi_0{ q_0[4] };  // steer at 0

    const double v_d{ u[0] };    // velocity desired
    const double phi_d{ u[1] };  // steering angle desired

    const double k_v{ 1 };    // velocity gain
    const double k_phi{ 1 };  // steer gain

    const double x_dot{ qdot[0] };  // x velocity
    const double y_dot{ qdot[1] };  // y velocity
    // const double phi_dot{ qdot[2] };  // phi velocity

    const double v_1{ std::sqrt(x_dot * x_dot + y_dot * y_dot) };  // velocity at 1

    Q q_1p{ Q::Zero() };

    q_1p.head(3) = q_0.head(3) + qdot * simulation_step;

    q_1p[3] = v_1 + k_v * (v_d - v_1);
    q_1p[4] = phi_0 + k_phi * (phi_0 - phi_d);

    q_1p[2] = norm_angle_pi(q_1p[2]);

    Q error{ Q::Zero() };
    error = q_1 - q_1p;
    // error[3] = v_1 - q_1p[3];
    return error;
  }

private:
  Partial_Q0 partial_q0;
  Partial_Q1 partial_q1;
  Partial_Qdot partial_qdot;
  Partial_U partial_u;

  mutable prx::math::first_order_derivative_t<Partial_Q0, Q, 4> derivative_q0;
  mutable prx::math::first_order_derivative_t<Partial_Q1, Q, 4> derivative_q1;
  mutable prx::math::first_order_derivative_t<Partial_Qdot, Qdot, 4> derivative_qdot;
  mutable prx::math::first_order_derivative_t<Partial_U, U, 4> derivative_u;
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
    // const double mu{ params[1] };
    const double mu{ std::exp(-environment_params[0] * 0.1) };

    const double f{ force[0] };

    const double theta{ q[2] };
    const double r_icc{ _wheel_distance / std::cos(theta) };

    Qdotdot qdotdot_p{ Qdotdot::Zero() };

    qdotdot_p[0] = mu * f * std::cos(theta) / _mass;
    qdotdot_p[1] = mu * f * std::sin(theta) / _mass;
    qdotdot_p[2] = (r_icc / I) * f;
    return qdotdot - qdotdot_p;
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
class ackermann_q_observation_t : public gtsam::NoiseModelFactor2<ackermann::Qz, ackermann::Q>
{
  using Q = ackermann::Q;
  using Qz = ackermann::Qz;

  using Base = gtsam::NoiseModelFactor2<Qz, Q>;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

  using Partial_Q = std::function<Qz(const Q&)>;
  using Partial_Qz = std::function<Qz(const Qz&)>;

public:
  ackermann_q_observation_t(const gtsam::Key key_qz, const gtsam::Key key_q, const NoiseModel& cost_model,
                            const double h = prx::simulation_step)
    : Base(cost_model, key_qz, key_q), derivative_qz(h), derivative_q(h)
  {
  }

  virtual Eigen::VectorXd evaluateError(const Qz& qz, const Q& q,                              // no-lint
                                        boost::optional<Eigen::MatrixXd&> H_qz = boost::none,  // no-lint
                                        boost::optional<Eigen::MatrixXd&> H_q = boost::none) const override
  {
    if (H_qz)
    {
      derivative_qz._model = [&](const Qz& qz_) { return compute_error(qz_, q); };
      *H_qz = derivative_qz(qz);
    }
    if (H_q)
    {
      derivative_q._model = [&](const Q& q_) { return compute_error(qz, q_); };
      *H_q = derivative_q(q);
    }

    return compute_error(qz, q);
  }

  Qz compute_error(const Qz& qz, const Q& q) const
  {
    return qz - q.head(3);
  }

private:
  Partial_Qz partial_qz;
  Partial_Q partial_q;

  mutable prx::math::first_order_derivative_t<Partial_Qz, Qz, 4> derivative_qz;
  mutable prx::math::first_order_derivative_t<Partial_Q, Q, 4> derivative_q;
};

// Factor <- Q observed aka (x,y,\theta)
// Error is Q - (x,y,\theta,0,0)
class ackermann_q_observation_t : public gtsam::NoiseModelFactor2<ackermann::Qz, ackermann::Q>
{
};

}  // namespace fg
}  // namespace prx