#pragma once

#include <Eigen/Dense>
#include <Eigen/Core>
#include <gtsam/base/OptionalJacobian.h>
#include "prx/factor_graphs/lie_groups/lie_operators.hpp"
#include "prx/factor_graphs/factors/noise_model_factors.hpp"
#include "prx/utilities/math/first_order_derivative.hpp"
#include "prx/factor_graphs/lie_groups/lie_integrator.hpp"

namespace prx
{
namespace fg
{
//: public noise_model_3factor_t<X, X, Xdot>
template <typename X, typename Xdot, typename... Types>
class lie_plan_integration_factor_t : public gtsam::NoiseModelFactorN<X, X, Xdot, Types...>
{
  using Base = gtsam::NoiseModelFactor3<X, X, Xdot, Types...>;
  using LieIntegrator = lie_integrator_t<X, Xdot, Types...>;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;
  static constexpr Eigen::Index DimX{ gtsam::traits<X>::dimension };
  static constexpr Eigen::Index DimXdot{ gtsam::traits<Xdot>::dimension };
  static constexpr std::size_t NumTypes{ sizeof...(Types) };

  using DerivativeX = Eigen::Matrix<double, DimX, DimX>;
  using OptDeriv = boost::optional<Eigen::MatrixXd&>;
  template <typename T>
  using OptionalMatrix = boost::optional<Eigen::MatrixXd&>;

  using MatXX = Eigen::Matrix<double, DimX, DimX>;
  using MatXXdot = Eigen::Matrix<double, DimX, DimXdot>;
  using MatXdotXdot = Eigen::Matrix<double, DimXdot, DimXdot>;
  using MatXdotDt = Eigen::Matrix<double, DimXdot, 1>;

  lie_plan_integration_factor_t() = delete;
  lie_plan_integration_factor_t(const lie_plan_integration_factor_t& other) = delete;

public:
  template <std::size_t Num = NumTypes, typename std::enable_if_t<(0 == Num), bool> = true>
  lie_plan_integration_factor_t(const gtsam::Key key_xt1, const gtsam::Key key_xt0, const gtsam::Key key_xdot,
                                const NoiseModel& cost_model, const double h,
                                const std::string label = "LieOdeIntegration")
    : Base(cost_model, key_xt1, key_xt0, key_xdot), _h(h), _label(label)
  {
  }

  template <std::size_t Num = NumTypes, typename std::enable_if_t<(1 == Num), bool> = true>
  lie_plan_integration_factor_t(const gtsam::Key key_xt1, const gtsam::Key key_xt0, const gtsam::Key key_xdot,
                                const gtsam::Key key_dt, const NoiseModel& cost_model,
                                const std::string label = "LieOdeIntegration")
    : Base(cost_model, key_xt1, key_xt0, key_xdot, key_dt), _h(0.0), _label(label)
  {
  }

  ~lie_plan_integration_factor_t() override
  {
  }

  // virtual X0 predict(const X1& x1, const X2& x2) const = 0;
  template <typename Dt>
  static X predict(const X& x, const Xdot& xdot, const Dt dt, const double h,   // no-lint
                   gtsam::OptionalJacobian<DimX, DimX> Hx = boost::none,        // no-lint
                   gtsam::OptionalJacobian<DimX, DimXdot> Hxdot = boost::none,  // no-lint
                   gtsam::OptionalJacobian<DimX, 1> Hdt = boost::none)
  {
    X xt{ x };
    for (double ti = 0.0; ti < dt; ti += h)
    {
      // xt = LieIntegrator::integrate(xt, xdot, dt, Hx, Hxdot, Hdt);
    }

    return xt;
  }

  // x1_predicted <- x0 + xdot dt
  // Error is: x1_predicted - x1_observed
  template <typename Dt>
  static Eigen::VectorXd error(const X& x1, const X& x0, const Xdot& xdot, const Dt& dt,
                               boost::optional<Eigen::MatrixXd&> Hx1 = boost::none,
                               boost::optional<Eigen::MatrixXd&> Hx0 = boost::none,
                               boost::optional<Eigen::MatrixXd&> Hxdot = boost::none,
                               boost::optional<Eigen::MatrixXd&> Hdt = boost::none)
  {
    Eigen::Matrix<double, DimX, DimX> err_H_b;       // Deriv error wrt between
    Eigen::Matrix<double, DimX, DimX> b_H_q1;        // Deriv between wrt x1
    Eigen::Matrix<double, DimX, DimX> b_H_qp;        // Deriv between wrt predicted
    Eigen::Matrix<double, DimX, DimX> qp_H_q0;       // Deriv predicted wrt x0
    Eigen::Matrix<double, DimX, DimXdot> qp_H_qdot;  // Deriv predicted wrt xdot
    Eigen::Matrix<double, DimX, 1> qp_H_qdt;         // Deriv predicted wrt dt

    const X prediction{ predict(x0, xdot, dt,                  // no-lint
                                Hx0 ? &qp_H_q0 : nullptr,      // no-lint
                                Hxdot ? &qp_H_qdot : nullptr,  // no-lint
                                Hdt ? &qp_H_qdt : nullptr) };
    // X1_p (-) x1 => Eq. 26 from "A micro Lie theory [...]" https://arxiv.org/pdf/1812.01537.pdf
    const X between{ x1.between(prediction,                                 // no-lint
                                (Hx0 or Hxdot or Hdt) ? &b_H_q1 : nullptr,  // no-lint
                                (Hx0 or Hxdot or Hdt) ? &b_H_qp : nullptr) };
    const Eigen::VectorXd error{ X::Logmap(between, (Hx0 or Hxdot or Hdt) ? &err_H_b : nullptr) };

    if (Hx1)
    {
      *Hx1 = err_H_b * b_H_q1;
    }
    if (Hx0)
    {
      *Hx0 = err_H_b * b_H_qp * qp_H_q0;
    }
    if (Hxdot)
    {
      *Hxdot = err_H_b * b_H_qp * qp_H_qdot;
    }
    if (Hdt)
    {
      *Hdt = err_H_b * b_H_qp * qp_H_qdt;
    }

    return error;
  }

  virtual Eigen::VectorXd evaluateError(const X& x1, const X& x0, const Xdot& xdot, const Types&... xd,  // no-lint
                                        OptDeriv H1 = boost::none, OptDeriv H0 = boost::none,
                                        OptDeriv Hdot = boost::none, OptionalMatrix<Types>... H) const override
  {
    if constexpr (0 == NumTypes)
    {
      return error(x1, x0, xdot, _h, H1, H0, Hdot);
    }
    else
    {
      return error(x1, x0, xdot, xd..., H1, H0, Hdot, H...);
    }
  }

private:
  const double _h;
  const std::string _label;
  // const DerivativeX _negative_identity;
};

}  // namespace fg
}  // namespace prx
