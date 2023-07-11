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

namespace fbd  // free body dynamics
{

const Eigen::Index DimX{ 3 };
const Eigen::Index DimXdot{ 3 };
const Eigen::Index DimXdotdot{ 3 };
const Eigen::Index DimForce{ 3 };
const Eigen::Index DimP{ 3 };
const Eigen::Index DimPdot{ 3 };
const Eigen::Index DimL{ 3 };
const Eigen::Index DimLdot{ 3 };
const Eigen::Index DimW{ 3 };
const Eigen::Index DimWdot{ 3 };
const Eigen::Index DimParams{ 3 };

using X = Eigen::Vector<double, DimX>;
using Xdot = Eigen::Vector<double, DimXdot>;
using Xdotdot = Eigen::Vector<double, DimXdotdot>;
using P = Eigen::Vector<double, DimP>;          // linear momentum
using Force = Eigen::Vector<double, DimForce>;  // == \dot{P}

using Q = Eigen::Quaternion<double>;
using Qdot = Eigen::Quaternion<double>;

using W = Eigen::Vector<double, DimW>;
using Wdot = Eigen::Vector<double, DimWdot>;

using L = Eigen::Vector<double, DimL>;          // angular momentum
using Torque = Eigen::Vector<double, DimLdot>;  // == \dot{L}

using I = Eigen::Matrix3d;
using Idot = Eigen::Matrix3d;

using I_body = Eigen::Matrix3d;
using Params = Eigen::Vector<double, DimParams>;

const prx::math::S Evaluations{ 4 };
}  // namespace fbd

// Error on X
// template <typename X, typename Xdot>
// class propagation_euler_factor_t : public gtsam::NoiseModelFactor3<X, X, Xdot>
// {
//   using Base = gtsam::NoiseModelFactor3<X, X, Xdot>;
//   using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

//   using Partial_X0 = std::function<X(const X&)>;
//   using Partial_X1 = std::function<X(const X&)>;
//   using Partial_Xdot = std::function<X(const Xdot&)>;

// public:
//   propagation_euler_factor_t(const gtsam::Key key_x0, const gtsam::Key key_x1, const gtsam::Key key_xdot,
//                              const NoiseModel& cost_model, const double h = prx::simulation_step)
//     : Base(cost_model, key_x0, key_x1, key_xdot), derivative_x0(h), derivative_x1(h), derivative_xdot(h)
//   {
//   }

//   virtual Eigen::VectorXd evaluateError(const X& x0, const X& x1, const Xdot& xdot,            // no-lint
//                                         boost::optional<Eigen::MatrixXd&> H_x0 = boost::none,  // no-lint
//                                         boost::optional<Eigen::MatrixXd&> H_x1 = boost::none,  // no-lint
//                                         boost::optional<Eigen::MatrixXd&> H_xdot = boost::none) const override
//   {
//     if (H_x0)
//     {
//       derivative_x0._model = [&](const X& x0_) { return compute_error(x0_, x1, xdot); };
//     }
//     if (H_x1)
//     {
//       derivative_x1._model = [&](const X& x1_) { return compute_error(x0, x1_, xdot); };
//     }
//     if (H_xdot)
//     {
//       derivative_xdot._model = [&](const Xdot& xdot_) { return compute_error(x0, x1, xdot_); };
//     }

//     return compute_error(x0, x1, xdot);
//   }

//   X compute_error(const X& x0, const X& x1, const Xdot& xdot) const
//   {
//     return x1 - (x0 + xdot * simulation_step);
//   }

// private:
//   Partial_X0 partial_x0;
//   Partial_X1 partial_x1;
//   Partial_Xdot partial_xdot;

//   mutable prx::math::first_order_derivative_t<Partial_X0, X, 4> derivative_x0;
//   mutable prx::math::first_order_derivative_t<Partial_X1, X, 4> derivative_x1;
//   mutable prx::math::first_order_derivative_t<Partial_Xdot, Xdot, 4> derivative_xdot;
// };

class velocity_linear_momentum_factor_t : public noise_model_2factor_t<fbd::DimXdot, fbd::DimP>
{
  using Base = noise_model_2factor_t<fbd::DimXdot, fbd::DimP>;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

public:
  velocity_linear_momentum_factor_t(const gtsam::Key key_xdot, const gtsam::Key key_p, const NoiseModel& cost_model,
                                    const double mass, const double h = prx::simulation_step)
    : Base(key_xdot, key_p, cost_model, h), _mass(mass)
  {
  }

  virtual fbd::Xdot compute_error(const fbd::Xdot& xdot, const fbd::P& p) const override
  {
    return p - (_mass * xdot);
  }

private:
  const double _mass;
};

class force_acceleration_factor_t : public noise_model_2factor_t<fbd::DimXdotdot, fbd::DimForce>
{
  using Base = noise_model_2factor_t<fbd::DimXdotdot, fbd::DimForce>;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

public:
  force_acceleration_factor_t(const gtsam::Key key_xdotdot, const gtsam::Key key_force, const NoiseModel& cost_model,
                              const double mass, const double h = prx::simulation_step)
    : Base(key_xdotdot, key_force, cost_model, h), _mass(mass)
  {
  }

  virtual fbd::Xdot compute_error(const fbd::Xdotdot& xdotdot, const fbd::Force& force) const override
  {
    return force - (_mass * xdotdot);
  }

private:
  const double _mass;
};

class angular_velocity_factor_t : public gtsam::NoiseModelFactor3<fbd::Q, fbd::Qdot, fbd::W>
{
  using Base = gtsam::NoiseModelFactor3<fbd::Q, fbd::Qdot, fbd::W>;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

  using Qvec = Eigen::Vector<double, 4>;  // Check if this is correct
  using Partial_Q = std::function<Qvec(const Qvec&)>;
  using Partial_Qdot = std::function<Qvec(const Qvec&)>;
  using Partial_W = std::function<Qvec(const fbd::W&)>;

public:
  angular_velocity_factor_t(const gtsam::Key key_q, const gtsam::Key key_qdot, const gtsam::Key key_w,
                            const NoiseModel& cost_model, const double h = prx::simulation_step)
    : Base(cost_model, key_q, key_qdot, key_w), derivative_q(h), derivative_qdot(h), derivative_w(h)
  {
  }

  virtual Eigen::VectorXd evaluateError(const fbd::Q& q, const fbd::Qdot& qdot, const fbd::W& w,  // no-lint
                                        boost::optional<Eigen::MatrixXd&> H_q = boost::none,      // no-lint
                                        boost::optional<Eigen::MatrixXd&> H_qdot = boost::none,   // no-lint
                                        boost::optional<Eigen::MatrixXd&> H_w = boost::none) const override
  {
    const Qvec q_quat{ q.w(), q.x(), q.y(), q.z() };
    const Qvec qdot_quat{ qdot.w(), qdot.x(), qdot.y(), qdot.z() };

    if (H_q)
    {
      derivative_q._model = [&](const Qvec& q_) { return compute_error(q_, qdot_quat, w); };
    }
    if (H_qdot)
    {
      derivative_qdot._model = [&](const Qvec& qdot_) { return compute_error(q_quat, qdot_, w); };
    }
    if (H_w)
    {
      derivative_w._model = [&](const fbd::W& w_) { return compute_error(q_quat, qdot_quat, w_); };
    }

    return compute_error(q_quat, qdot_quat, w);
  }

  Qvec compute_error(const Qvec& q, const Qvec& qdot, const fbd::W& w) const
  {
    // 0.5 * [0,\omega]
    const fbd::Q w_quat{ 0, 0.5 * w[0], 0.5 * w[1], 0.5 * w[2] };
    const fbd::Q q_quat{ q[0], q[1], q[2], q[3] };

    const fbd::Q q_res{ w_quat * q_quat };
    return qdot - Qvec{ q_res.w(), q_res.x(), q_res.y(), q_res.z() };
  }

private:
  Partial_Q partial_q;
  Partial_Qdot partial_qdot;
  Partial_W partial_w;

  mutable prx::math::first_order_derivative_t<Partial_Q, Qvec, 4> derivative_q;
  mutable prx::math::first_order_derivative_t<Partial_Qdot, Qvec, 4> derivative_qdot;
  mutable prx::math::first_order_derivative_t<Partial_W, fbd::W, 4> derivative_w;
};

// class angular_acceleration_factor_t
//   : public noise_model_5factor_t<fbd::DimWdot, fbd::DimL, fbd::DimLdot, fbd::DimI, fbd::DimIdot>
// {
//   using Base = noise_model_5factor_t<fbd::DimWdot, fbd::DimL, fbd::DimLdot, fbd::DimI, fbd::DimIdot>;
//   using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

// public:
//   angular_acceleration_factor_t(const gtsam::Key key_wdot, const gtsam::Key key_l, const gtsam::Key key_torque,
//                                 const gtsam::Key key_i, const gtsam::Key key_idot, const NoiseModel& cost_model,
//                                 const double h = prx::simulation_step)
//     : Base(key_wdot, key_l, key_torque, key_i, key_idot, cost_model, h)
//   {
//   }

//   virtual fbd::Xdot compute_error(const fbd::Wdot& wdot, const fbd::L& l, const fbd::Torque& torque, const fbd::I& i,
//                                   const fbd::Idot& idot) const override
//   {
//     return wdot - (Idot.inverse() * l + I.inverse() * torque);
//   }
// };

// class angular_momentum_factor_t : public noise_model_3factor_t<fbd::L, fbd::I, fbd::W>
// {
//   using Base = noise_model_3factor_t<fbd::DimL, fbd::DimI, fbd::DimW>;
//   using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

// public:
//   angular_momentum_factor_t(const gtsam::Key key_l, const gtsam::Key key_i, const gtsam::Key key_w,
//                             const NoiseModel& cost_model, const double h = prx::simulation_step)
//     : Base(key_l, key_i, key_w, cost_model, h)
//   {
//   }

//   virtual fbd::Xdot compute_error(const fbd::L& l, const fbd::I& i, const fbd::W& w) const override
//   {
//     return l - i * w;
//   }
// };

}  // namespace fg
}  // namespace prx
