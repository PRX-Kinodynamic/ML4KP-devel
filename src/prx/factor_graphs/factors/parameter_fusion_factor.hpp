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
template <Eigen::Index X_DIM, Eigen::Index THETA_DIM>
class parameter_fusion_factor_t
  : public gtsam::NoiseModelFactor5<Eigen::Vector<double, X_DIM + THETA_DIM>, Eigen::Vector<double, X_DIM + THETA_DIM>,
                                    Eigen::Vector<double, X_DIM + THETA_DIM>, Eigen::Vector<double, X_DIM + THETA_DIM>,
                                    Eigen::Vector<double, X_DIM + THETA_DIM>>
{
  template <typename T, Eigen::Index DIM>
  using partial_fn = std::function<Eigen::Vector<double, DIM>(const T&)>;
  using x_vector_t = Eigen::Vector<double, X_DIM>;
  using value_t = Eigen::Vector<double, THETA_DIM>;
  using Base = gtsam::NoiseModelFactor5<value_t, value_t, value_t, value_t, value_t>;

public:
  parameter_fusion_factor_t(gtsam::Key th0_key, gtsam::Key th1_key, gtsam::Key th2_key, gtsam::Key th3_key,
                            gtsam::Key thX_key, const gtsam::noiseModel::Base::shared_ptr& cost_model,
                            const x_vector_t& X_th0, const x_vector_t& X_th1, const x_vector_t& X_th2,
                            const x_vector_t& X_th3, const x_vector_t& X_thx)
    : Base(cost_model, th0_key, th1_key, th2_key, th3_key, thX_key)
    , derivative_th0(partial_th0, 0.01)
    , derivative_th1(partial_th1, 0.01)
    , derivative_th2(partial_th2, 0.01)
    , derivative_th3(partial_th3, 0.01)
    , derivative_thX(partial_thX, 0.01)
    , _X_th0(X_th0)
    , _X_th1(X_th1)
    , _X_th2(X_th2)
    , _X_th3(X_th3)
    , _X_thX(X_thX)
  {
  }

  virtual ~parameter_fusion_factor_t()
  {
  }

  virtual Eigen::VectorXd evaluateError(const value_t& th0, const value_t& th1, const value_t& th2, const value_t& th3,
                                        const value_t& thX, boost::optional<Eigen::MatrixXd&> H1 = boost::none,
                                        boost::optional<Eigen::MatrixXd&> H2 = boost::none,
                                        boost::optional<Eigen::MatrixXd&> H3 = boost::none,
                                        boost::optional<Eigen::MatrixXd&> H4 = boost::none,
                                        boost::optional<Eigen::MatrixXd&> H5 = boost::none) const override
  {
    auto error = compute_error(th0, th1, th2, th3, thX);
    if (H1)
    {
      derivative_th0.model = [&](const value_t& _th0) { return compute_error(_th0, th1, th2, th3, thX); };
      *H1 = derivative_th0(th0);
    }

    if (H2)
    {
      derivative_th1.model = [&](const value_t& _th1) { return compute_error(th0, _th1, th2, th3, thX); };
      *H2 = derivative_th1(th1);
    }

    if (H3)
    {
      derivative_th2.model = [&](const value_t& _th2) { return compute_error(th0, th1, _th2, th3, thX); };
      *H3 = derivative_th2(th0);
    }

    if (H4)
    {
      derivative_th3.model = [&](const value_t& _th3) { return compute_error(th0, th1, th2, _th3, thX); };
      *H4 = derivative_th3(th3);
    }

    if (H5)
    {
      derivative_thX.model = [&](const value_t& _thX) { return compute_error(th0, th1, th2, th3, _thX); };
      *H5 = derivative_thX(thX);
    }

    return error;
  }

  double th_dist(const x_value_t& thi, const x_value_t& thX) const
  {
    return (thi - thX).squaredNorm();
  }

  Eigen::VectorXd compute_error(const value_t& th0, const value_t& th1, const value_t& th2, const value_t& th3,
                                const value_t& thX) const
  {
    const double dX0{ th_dist(_X_th0, _X_thX) };
    const double dX1{ th_dist(_X_th1, _X_thX) };
    const double dX2{ th_dist(_X_th2, _X_thX) };
    const double dX3{ th_dist(_X_th3, _X_thX) };

    Eigen::VectorXd error(Eigen::VectorXd::Zero(THETA_DIM));
    error += (th0.tail(THETA_DIM) - thX.tail(THETA_DIM)) / dX0;
    error += (th1.tail(THETA_DIM) - thX.tail(THETA_DIM)) / dX1;
    error += (th2.tail(THETA_DIM) - thX.tail(THETA_DIM)) / dX2;
    error += (th3.tail(THETA_DIM) - thX.tail(THETA_DIM)) / dX3;
    return error;
  }

private:
  x_vector_t _X_th0;
  x_vector_t _X_th1;
  x_vector_t _X_th2;
  x_vector_t _X_th3;
  x_vector_t _X_thX;

  partial_fn<value_t, THETA_DIM> partial_th0;
  partial_fn<value_t, THETA_DIM> partial_th1;
  partial_fn<value_t, THETA_DIM> partial_th2;
  partial_fn<value_t, THETA_DIM> partial_th3;
  partial_fn<value_t, THETA_DIM> partial_thX;

  mutable math::first_order_derivative_t<partial_fn<value_t, THETA_DIM>, value_t, 4> derivative_th0;
  mutable math::first_order_derivative_t<partial_fn<value_t, THETA_DIM>, value_t, 4> derivative_th1;
  mutable math::first_order_derivative_t<partial_fn<value_t, THETA_DIM>, value_t, 4> derivative_th2;
  mutable math::first_order_derivative_t<partial_fn<value_t, THETA_DIM>, value_t, 4> derivative_th3;
  mutable math::first_order_derivative_t<partial_fn<value_t, THETA_DIM>, value_t, 4> derivative_thX;
};

}  // namespace fg
}  // namespace prx