#pragma once
#include <array>
#include <numeric>
#include <functional>

#include <gtsam/config.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "prx/factor_graphs/utilities/symbols_factory.hpp"
#include "prx/utilities/math/first_order_derivative.hpp"
#include "prx/simulation/system_factory.hpp"

namespace prx
{
namespace fg
{
template <typename X, typename Ctrl, typename... Types>
class prx_propagation_factor_t : public gtsam::NoiseModelFactorN<X, X, Ctrl, Types...>
{
  using Base = gtsam::NoiseModelFactorN<X, X, Ctrl, Types...>;
  using Derived = prx_propagation_factor_t<X, Ctrl, Types...>;

  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

  static constexpr Eigen::Index DimX{ gtsam::traits<X>::dimension };
  static constexpr Eigen::Index DimU{ gtsam::traits<Ctrl>::dimension };
  static constexpr std::size_t NumTypes{ sizeof...(Types) };

  using OptDeriv = boost::optional<Eigen::MatrixXd&>;
  template <typename T>
  using OptionalMatrix = boost::optional<Eigen::MatrixXd&>;

  using Error = Eigen::Vector<double, DimX>;
  template <typename Input>
  using Partial = std::function<Error(const Input&)>;
  template <typename Input>
  using FirstOrderDerivative = prx::math::first_order_derivative_t<Partial<Input>, Input, 4>;

  prx_propagation_factor_t() = delete;

public:
  prx_propagation_factor_t(const prx_propagation_factor_t& other) = delete;

  // TODO: Initialize system from copy / init?
  template <std::size_t Num = NumTypes, typename std::enable_if_t<(0 == Num), bool> = true>
  prx_propagation_factor_t(const gtsam::Key key_xt1, const gtsam::Key key_xt0, const gtsam::Key key_xdot,
                           const NoiseModel& cost_model, const double dt, const std::string plant_name)
    : Base(cost_model, key_xt1, key_xt0, key_xdot)
    , _dt(dt)
    , _prx_system(prx::system_factory_t::create_system(plant_name, plant_name))
    , _x1_ptr()
    , _x0_ptr()
    , _u_ptr()
    , _dt_ptr()
    , _partial_x1([&](const X& x1) { return error_(x1, predict_(*_x0_ptr, *_u_ptr, dt)); })
    , _partial_x0([&](const X& x0) { return predict_(x0, *_u_ptr, dt); })
    , _partial_u([&](const Ctrl& u) { return predict_(*_x0_ptr, u, dt); })
    , _partial_dt([&](const double& dt) {
      prx_throw("PartialDt shouldn't be called");
      return Error::Zero();
    })
    , _derivative_x1(_partial_x1, dt, DimX, DimX)
    , _derivative_x0(_partial_x0, dt, DimX, DimX)
    , _derivative_u(_partial_u, dt, DimU, DimX)
    , _derivative_dt(_partial_dt, dt, 1, DimX)
  {
  }

  template <std::size_t Num = NumTypes, typename std::enable_if_t<(1 == Num), bool> = true>
  prx_propagation_factor_t(const gtsam::Key key_xt1, const gtsam::Key key_xt0, const gtsam::Key key_u,
                           const gtsam::Key key_dt, const NoiseModel& cost_model, const std::string plant_name,
                           const double h = 0.01)
    : Base(cost_model, key_xt1, key_xt0, key_u, key_dt)
    , _dt(0.0)
    , _prx_system(prx::system_factory_t::create_system(plant_name, plant_name))
    , _x1_ptr()
    , _x0_ptr()
    , _u_ptr()
    , _dt_ptr()
    , _partial_x1([&](const X& x1) { return error_(x1, predict_(*_x0_ptr, *_u_ptr, *_dt_ptr)); })
    , _partial_x0([&](const X& x0) { return predict_(x0, *_u_ptr, *_dt_ptr); })
    , _partial_u([&](const Ctrl& u) { return predict_(*_x0_ptr, u, *_dt_ptr); })
    , _partial_dt([&](const double& dt) { return predict_(*_x0_ptr, *_u_ptr, dt); })
    , _derivative_x1(_partial_x1, h, DimX, DimX)
    , _derivative_x0(_partial_x0, h, DimX, DimX)
    , _derivative_u(_partial_u, h, DimU, DimX)
    , _derivative_dt(_partial_dt, h, 1, DimX)
  {
  }

  ~prx_propagation_factor_t() override
  {
  }

  template <typename Dt>
  inline X integrate(const X& x, const Ctrl& u, const Dt& dt,  // no-lint
                     OptDeriv Hx = boost::none, OptDeriv Hu = boost::none, OptDeriv Hdt = boost::none)
  {
    return predict(x, u, dt, Hx, Hu, Hdt);
  }

  template <typename Dt>
  X predict(const X& x0, const Ctrl& u, const Dt& dt,  // no-lint
            OptDeriv Hx = boost::none, OptDeriv Hu = boost::none, OptDeriv Hdt = boost::none) const
  {
    const X prediction{ predict_(x0, u, dt) };
    compute_derivatives(x0, x0, u, dt, boost::none, Hx, Hu, Hdt);

    return prediction;
  }

  template <typename Dt>
  X error(const X& x1, const X& x0, const Ctrl& u, const Dt& dt,  // no-lint
          OptDeriv H1 = boost::none, OptDeriv H0 = boost::none, OptDeriv Hu = boost::none,
          OptDeriv Hdt = boost::none) const
  {
    const X prediction{ predict(x0, u, dt) };
    const Error error{ error_(x1, prediction) };
    compute_derivatives(x1, x0, u, dt, H1, H0, Hu, Hdt);

    return error;
  }

  // template <std::enable_if_t<not DtKey, bool> = true>
  virtual Eigen::VectorXd evaluateError(const X& x1, const X& x0, const Ctrl& u, const Types&... xd,  // no-lint
                                        OptDeriv H1 = boost::none, OptDeriv H0 = boost::none, OptDeriv Hu = boost::none,
                                        OptionalMatrix<Types>... H) const override
  {
    if constexpr (0 == NumTypes)
    {
      return error(x1, x0, u, _dt, H1, H0, Hu);
    }
    else
    {
      return error(x1, x0, u, xd..., H1, H0, Hu, H...);
    }
  }

  virtual X error_(const X& x1, const X& prediction) const
  {
    const X error{ prediction - x1 };
    return error;
  }

  virtual X predict_(const X& x0, const Ctrl& u, const double& dt) const
  {
    _prx_system->get_state_space()->copy_from(x0);
    _prx_system->get_control_space()->copy_from(u);
    for (double i = 0; i < dt; i += prx::simulation_step)
    {
      _prx_system->propagate(prx::simulation_step);
    }
    _prx_system->get_state_space()->copy_to(_state);
    return _state;
  }

protected:
  template <typename Dt>
  void compute_derivatives(const X& x1, const X& x0, const Ctrl& u, const Dt& dt,  // no-lint
                           OptDeriv H1 = boost::none, OptDeriv H0 = boost::none, OptDeriv Hu = boost::none,
                           OptDeriv Hdt = boost::none) const
  {
    _x0_ptr.reset(&x0);
    _u_ptr.reset(&u);
    _dt_ptr.reset(&dt);
    _x1_ptr.reset(&x1);

    if (H1)
    {
      *H1 = _derivative_x1(x1);
    }
    if (H0)
    {
      *H0 = _derivative_x0(x0);
    }
    if (Hu)
    {
      *Hu = _derivative_u(u);
    }
    if constexpr (NumTypes == 1)
    {
      if (Hdt)
      {
        *Hdt = _derivative_dt(dt);
      }
    }

    _x1_ptr.release();
    _x0_ptr.release();
    _u_ptr.release();
    _dt_ptr.release();
  }

  mutable X _state;
  const double _dt;
  mutable prx::system_ptr_t _prx_system;

  mutable std::unique_ptr<const X> _x1_ptr;
  mutable std::unique_ptr<const X> _x0_ptr;
  mutable std::unique_ptr<const Ctrl> _u_ptr;
  mutable std::unique_ptr<const double> _dt_ptr;

  const Partial<X> _partial_x1;
  const Partial<X> _partial_x0;
  const Partial<Ctrl> _partial_u;
  const Partial<double> _partial_dt;

  const FirstOrderDerivative<X> _derivative_x1;
  const FirstOrderDerivative<X> _derivative_x0;
  const FirstOrderDerivative<Ctrl> _derivative_u;
  const FirstOrderDerivative<double> _derivative_dt;
};

}  // namespace fg
}  // namespace prx