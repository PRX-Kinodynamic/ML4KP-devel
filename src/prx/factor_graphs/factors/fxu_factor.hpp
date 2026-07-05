#pragma once
#include <gtsam/config.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace prx
{
namespace fg
{

template <typename State, typename Control, typename PlantPointer>
class fxu_factor_t : public gtsam::NoiseModelFactorN<State, Control, State>
{
  using Base = gtsam::NoiseModelFactorN<State, Control, State>;
  using Derived = fxu_factor_t<State, Control, PlantPointer>;

  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

  using OptDeriv = boost::optional<Eigen::MatrixXd&>;

  static constexpr Eigen::Index DimX{ gtsam::traits<State>::dimension };
  static constexpr Eigen::Index DimU{ gtsam::traits<Control>::dimension };

  fxu_factor_t() = delete;

public:
  fxu_factor_t(const fxu_factor_t& other) = delete;

  fxu_factor_t(const gtsam::Key key_x0, const gtsam::Key key_u, const gtsam::Key key_x1, const NoiseModel& cost_model,
               const double dt, PlantPointer plant)
    : Base(cost_model, key_x0, key_u, key_x1), _dt(dt), _plant(plant)
  {
  }

  ~fxu_factor_t() override
  {
  }

  // State predict(const State& x, const Control& u, const double& dt,  // no-lint
  //               Eigen::Matrix<double, DimX, DimX>* Hx = nullptr,     // no-lint
  //               Eigen::Matrix<double, DimX, DimU>* Hu = nullptr)
  // {
  //   return _plant->propagate(x, u, dt, Hx, Hu);
  // }

  // Error is: z (-) q_^{predicted}_1; where q_^{predicted}_1 = q0 (+) qdot dt, for a fix (known) dt
  virtual Eigen::VectorXd evaluateError(const State& x0, const Control& u0, const State& x1,  // no-lint
                                        OptDeriv Hx0 = boost::none,                           // no-lint
                                        OptDeriv Hu0 = boost::none,                           // no-lint
                                        OptDeriv Hx1 = boost::none) const override
  {
    using JacXX = Eigen::Matrix<double, DimX, DimX>;
    using JacXU = Eigen::Matrix<double, DimX, DimU>;

    const bool compute_jacs{ Hx0 or Hu0 or Hx1 };

    JacXX xbtw_H_x1{ JacXX::Identity() };
    JacXX xbtw_H_x1p{ JacXX::Identity() };
    JacXX x1p_H_x0{ JacXX::Identity() };
    JacXU x1p_H_u0{ JacXU::Identity() };
    JacXX err_H_xbtw{ JacXX::Identity() };

    const State x1p{ _plant->propagate(x0, u0, _dt,                         // no-lint
                                       compute_jacs ? &x1p_H_x0 : nullptr,  // no-lint
                                       compute_jacs ? &x1p_H_u0 : nullptr) };

    const State xbtw{ gtsam::traits<State>::Between(x1, x1p,                              // no-lint
                                                    compute_jacs ? &xbtw_H_x1 : nullptr,  // no-lint
                                                    compute_jacs ? &xbtw_H_x1p : nullptr) };
    const Eigen::Vector<double, DimX> err{ gtsam::traits<State>::Logmap(xbtw, compute_jacs ? &err_H_xbtw : nullptr) };

    if (Hx0)
    {
      *Hx0 = err_H_xbtw * xbtw_H_x1p * x1p_H_x0;
    }
    if (Hu0)
    {
      *Hu0 = err_H_xbtw * xbtw_H_x1p * x1p_H_u0;
    }
    if (Hx1)
    {
      *Hx1 = err_H_xbtw * xbtw_H_x1;
    }
    return err;
  }

private:
  const double _dt;
  PlantPointer _plant;
};
}  // namespace fg
}  // namespace prx