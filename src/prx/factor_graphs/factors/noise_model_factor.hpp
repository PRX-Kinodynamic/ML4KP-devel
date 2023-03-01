#pragma once
#include <array>
#include <numeric>
#include <functional>

#include <gtsam/config.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "prx/utilities/math/first_order_derivative.hpp"

using Evals = uint8_t;
namespace prx
{
namespace fg
{
/**
 * @brief      Implements \f$ \mathtt{error\_vector}(x) = h(X0)
 *
 * @tparam     Dim_X0  { Dimention of the vector X0 }
 * @tparam     Evaluations  { Number of evaluations to perform in the numerical derivative. See
 * first_order_derivative_t. }
 */
template <Eigen::Index Dim_X0, Evals Evaluations = 4>
class noise_model_1factor_t : public gtsam::NoiseModelFactor1<Eigen::Vector<double, Dim_X0>>
{
protected:
  using X0 = Eigen::Vector<double, Dim_X0>;
  using partial_X0 = std::function<Eigen::Vector<double, Dim_X0>(const X0&)>;
  using Base = gtsam::NoiseModelFactor1<X0>;

public:
  noise_model_1factor_t(gtsam::Key key_x0, const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(cost_model, key_x0), derivative_x0(partial_x0, 0.01)
  {
  }

  virtual ~noise_model_1factor_t()
  {
  }

  virtual Eigen::VectorXd evaluateError(const X0& x0,  // no-lint
                                        boost::optional<Eigen::MatrixXd&> H0 = boost::none) const override
  {
    auto error = compute_error(x0);
    if (H0)
    {
      derivative_x0.model = [&](const X0& _x0) { return compute_error(_x0); };
      *H0 = derivative_x0(x0);
    }

    return error;
  }

  virtual X0 compute_error(const X0& x0) const = 0;

private:
  partial_X0 partial_x0;

  mutable prx::math::first_order_derivative_t<partial_X0, X0, Evaluations> derivative_x0;
};

/**
 * @brief      Implements \f$ \mathtt{error\_vector}(x) = X0 - h(X1)
 *
 * @tparam     Dim_X0  { Dimention of the vector X0 }
 * @tparam     Dim_X1  { Dimention of the vector X1 }
 * @tparam     Dim_X2  { Dimention of the vector X2 }
 * @tparam     Evaluations  { Number of evaluations to perform in the numerical derivative. See
 * first_order_derivative_t. }
 */
template <Eigen::Index Dim_X0, Eigen::Index Dim_X1, Evals Evaluations = 4>
class noise_model_2factor_t
  : public gtsam::NoiseModelFactor2<Eigen::Vector<double, Dim_X0>, Eigen::Vector<double, Dim_X1>>
{
protected:
  using X0 = Eigen::Vector<double, Dim_X0>;
  using X1 = Eigen::Vector<double, Dim_X1>;
  using partial_X0 = std::function<Eigen::Vector<double, Dim_X0>(const X0&)>;
  using partial_X1 = std::function<Eigen::Vector<double, Dim_X0>(const X1&)>;
  using Base = gtsam::NoiseModelFactor2<X0, X1>;

public:
  noise_model_2factor_t(gtsam::Key key_x0, gtsam::Key key_x1, const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(cost_model, key_x0, key_x1), derivative_x0(partial_x0, 0.01), derivative_x1(partial_x1, 0.01)
  {
  }

  virtual ~noise_model_2factor_t()
  {
  }

  virtual Eigen::VectorXd evaluateError(const X0& x0, const X1& x1,  // no-lint
                                        boost::optional<Eigen::MatrixXd&> H0 = boost::none,
                                        boost::optional<Eigen::MatrixXd&> H1 = boost::none) const override
  {
    auto error = compute_error(x0, x1);
    if (H0)
    {
      derivative_x0.model = [&](const X0& _x0) { return compute_error(_x0, x1); };
      *H0 = derivative_x0(x0);
    }

    if (H1)
    {
      derivative_x1.model = [&](const X1& _x1) { return compute_error(x0, _x1); };
      *H1 = derivative_x1(x1);
    }

    return error;
  }

  virtual X0 compute_error(const X0& x0, const X1& x1) const = 0;

private:
  partial_X0 partial_x0;
  partial_X1 partial_x1;

  mutable prx::math::first_order_derivative_t<partial_X0, X0, Evaluations> derivative_x0;
  mutable prx::math::first_order_derivative_t<partial_X1, X1, Evaluations> derivative_x1;
};

/**
 * @brief      Implements \f$ \mathtt{error\_vector}(x) = X0 - h(X1,X2)
 *
 * @tparam     Dim_X0  { Dimention of the vector X0 }
 * @tparam     Dim_X1  { Dimention of the vector X1 }
 * @tparam     Dim_X2  { Dimention of the vector X2 }
 * @tparam     Evaluations  { Number of evaluations to perform in the numerical derivative. See
 * first_order_derivative_t. }
 */
template <Eigen::Index Dim_X0, Eigen::Index Dim_X1, Eigen::Index Dim_X2, Evals Evaluations = 4>
class noise_model_3factor_t
  : public gtsam::NoiseModelFactor3<Eigen::Vector<double, Dim_X0>, Eigen::Vector<double, Dim_X1>,
                                    Eigen::Vector<double, Dim_X2>>
{
protected:
  using X0 = Eigen::Vector<double, Dim_X0>;
  using X1 = Eigen::Vector<double, Dim_X1>;
  using X2 = Eigen::Vector<double, Dim_X2>;
  using partial_X0 = std::function<Eigen::Vector<double, Dim_X0>(const X0&)>;
  using partial_X1 = std::function<Eigen::Vector<double, Dim_X0>(const X1&)>;
  using partial_X2 = std::function<Eigen::Vector<double, Dim_X0>(const X2&)>;
  using Base = gtsam::NoiseModelFactor3<X0, X1, X2>;

public:
  noise_model_3factor_t(gtsam::Key key_x0, gtsam::Key key_x1, gtsam::Key key_x2,
                        const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(cost_model, key_x0, key_x1, key_x2)
    , derivative_x0(partial_x0, 0.01)
    , derivative_x1(partial_x1, 0.01)
    , derivative_x2(partial_x2, 0.01)
  {
  }

  virtual ~noise_model_3factor_t()
  {
  }

  virtual Eigen::VectorXd evaluateError(const X0& x0, const X1& x1, const X2& x2,
                                        boost::optional<Eigen::MatrixXd&> H0 = boost::none,
                                        boost::optional<Eigen::MatrixXd&> H1 = boost::none,
                                        boost::optional<Eigen::MatrixXd&> H2 = boost::none) const override
  {
    auto error = compute_error(x0, x1, x2);
    if (H0)
    {
      derivative_x0.model = [&](const X0& _x0) { return compute_error(_x0, x1, x2); };
      *H0 = derivative_x0(x0);
    }

    if (H1)
    {
      derivative_x1.model = [&](const X1& _x1) { return compute_error(x0, _x1, x2); };
      *H1 = derivative_x1(x1);
    }

    if (H2)
    {
      derivative_x2.model = [&](const X2& _x2) { return compute_error(x0, x1, _x2); };
      *H2 = derivative_x2(x2);
    }

    return error;
  }

  virtual X0 compute_error(const X0& x0, const X1& x1, const X2& x2) const = 0;

private:
  partial_X0 partial_x0;
  partial_X1 partial_x1;
  partial_X2 partial_x2;

  mutable prx::math::first_order_derivative_t<partial_X0, X0, Evaluations> derivative_x0;
  mutable prx::math::first_order_derivative_t<partial_X1, X1, Evaluations> derivative_x1;
  mutable prx::math::first_order_derivative_t<partial_X2, X2, Evaluations> derivative_x2;
};

/**
 * @brief      Implements \f$ \mathtt{error\_vector}(x) = X0 - h(X1, X2, X3)
 *
 * @tparam     Dim_X0  { Dimention of the vector X0 }
 * @tparam     Dim_X1  { Dimention of the vector X1 }
 * @tparam     Dim_X2  { Dimention of the vector X2 }
 * @tparam     Dim_X3  { Dimention of the vector X3 }
 * @tparam     Evaluations  { Number of evaluations to perform in the numerical derivative. See
 * first_order_derivative_t. }
 */
template <Eigen::Index Dim_X0, Eigen::Index Dim_X1, Eigen::Index Dim_X2, Eigen::Index Dim_X3, Evals Evaluations = 4>
class noise_model_4factor_t
  : public gtsam::NoiseModelFactor4<Eigen::Vector<double, Dim_X0>, Eigen::Vector<double, Dim_X1>,
                                    Eigen::Vector<double, Dim_X2>, Eigen::Vector<double, Dim_X3>>
{
protected:
  using X0 = Eigen::Vector<double, Dim_X0>;
  using X1 = Eigen::Vector<double, Dim_X1>;
  using X2 = Eigen::Vector<double, Dim_X2>;
  using X3 = Eigen::Vector<double, Dim_X3>;
  using partial_X0 = std::function<Eigen::Vector<double, Dim_X0>(const X0&)>;
  using partial_X1 = std::function<Eigen::Vector<double, Dim_X0>(const X1&)>;
  using partial_X2 = std::function<Eigen::Vector<double, Dim_X0>(const X2&)>;
  using partial_X3 = std::function<Eigen::Vector<double, Dim_X0>(const X3&)>;
  using Base = gtsam::NoiseModelFactor4<X0, X1, X2, X3>;

public:
  noise_model_4factor_t(gtsam::Key key_x0, gtsam::Key key_x1, gtsam::Key key_x2, gtsam::Key key_x3,
                        const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(cost_model, key_x0, key_x1, key_x2, key_x3)
    , derivative_x0(partial_x0, 0.01)
    , derivative_x1(partial_x1, 0.01)
    , derivative_x2(partial_x2, 0.01)
    , derivative_x3(partial_x3, 0.01)
    , computing_derivative(false)
  {
  }

  virtual ~noise_model_4factor_t()
  {
  }

  virtual Eigen::VectorXd evaluateError(const X0& x0, const X1& x1, const X2& x2, const X3& x3,
                                        boost::optional<Eigen::MatrixXd&> H0 = boost::none,
                                        boost::optional<Eigen::MatrixXd&> H1 = boost::none,
                                        boost::optional<Eigen::MatrixXd&> H2 = boost::none,
                                        boost::optional<Eigen::MatrixXd&> H3 = boost::none) const override
  {
    computing_derivative = false;
    auto error = compute_error(x0, x1, x2, x3);
    computing_derivative = true;
    if (H0)
    {
      derivative_x0.model = [&](const X0& _x0) { return compute_error(_x0, x1, x2, x3); };
      *H0 = derivative_x0(x0);
    }

    if (H1)
    {
      derivative_x1.model = [&](const X1& _x1) { return compute_error(x0, _x1, x2, x3); };
      *H1 = derivative_x1(x1);
    }

    if (H2)
    {
      derivative_x2.model = [&](const X2& _x2) { return compute_error(x0, x1, _x2, x3); };
      *H2 = derivative_x2(x2);
    }

    if (H3)
    {
      derivative_x3.model = [&](const X3& _x3) { return compute_error(x0, x1, x2, _x3); };
      *H3 = derivative_x3(x3);
    }
    return error;
  }

  virtual X0 compute_error(const X0& x0, const X1& x1, const X2& x2, const X3& x3) const = 0;

private:
  partial_X0 partial_x0;
  partial_X1 partial_x1;
  partial_X2 partial_x2;
  partial_X3 partial_x3;

  mutable prx::math::first_order_derivative_t<partial_X0, X0, Evaluations> derivative_x0;
  mutable prx::math::first_order_derivative_t<partial_X1, X1, Evaluations> derivative_x1;
  mutable prx::math::first_order_derivative_t<partial_X2, X2, Evaluations> derivative_x2;
  mutable prx::math::first_order_derivative_t<partial_X3, X3, Evaluations> derivative_x3;

protected:
  mutable bool computing_derivative;  // Useful for debuging: avoid printing/stepping if compute_error is being used by
                                      // the derivative
};

/**
 * @brief      Implements \f$ \mathtt{error\_vector} = x0 - h(x1, x2, x3, x4)
 *
 * @tparam     Dim_X0  { Dimention of the vector X0 }
 * @tparam     Dim_X1  { Dimention of the vector X1 }
 * @tparam     Dim_X2  { Dimention of the vector X2 }
 * @tparam     Dim_X3  { Dimention of the vector X3 }
 * @tparam     Dim_X4  { Dimention of the vector X4 }
 * @tparam     Evaluations  { Number of evaluations to perform in the numerical derivative. See
 * first_order_derivative_t. }
 */
template <Eigen::Index Dim_X0, Eigen::Index Dim_X1, Eigen::Index Dim_X2, Eigen::Index Dim_X3, Eigen::Index Dim_X4,
          Evals Evaluations = 4>
class noise_model_5factor_t
  : public gtsam::NoiseModelFactor5<Eigen::Vector<double, Dim_X0>, Eigen::Vector<double, Dim_X1>,
                                    Eigen::Vector<double, Dim_X2>, Eigen::Vector<double, Dim_X3>,
                                    Eigen::Vector<double, Dim_X4>>
{
protected:
  using X0 = Eigen::Vector<double, Dim_X0>;
  using X1 = Eigen::Vector<double, Dim_X1>;
  using X2 = Eigen::Vector<double, Dim_X2>;
  using X3 = Eigen::Vector<double, Dim_X3>;
  using X4 = Eigen::Vector<double, Dim_X4>;
  using partial_X0 = std::function<Eigen::Vector<double, Dim_X0>(const X0&)>;
  using partial_X1 = std::function<Eigen::Vector<double, Dim_X0>(const X1&)>;
  using partial_X2 = std::function<Eigen::Vector<double, Dim_X0>(const X2&)>;
  using partial_X3 = std::function<Eigen::Vector<double, Dim_X0>(const X3&)>;
  using partial_X4 = std::function<Eigen::Vector<double, Dim_X0>(const X4&)>;
  using Base = gtsam::NoiseModelFactor5<X0, X1, X2, X3, X4>;

public:
  noise_model_5factor_t(gtsam::Key key_x0, gtsam::Key key_x1, gtsam::Key key_x2, gtsam::Key key_x3, gtsam::Key key_x4,
                        const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(cost_model, key_x0, key_x1, key_x2, key_x3, key_x4)
    , derivative_x0(partial_x0, 0.01)
    , derivative_x1(partial_x1, 0.01)
    , derivative_x2(partial_x2, 0.01)
    , derivative_x3(partial_x3, 0.01)
    , derivative_x4(partial_x4, 0.01)
  {
  }

  virtual ~noise_model_5factor_t()
  {
  }

  virtual Eigen::VectorXd evaluateError(const X0& x0, const X1& x1, const X2& x2, const X3& x3, const X4& x4,
                                        boost::optional<Eigen::MatrixXd&> H0 = boost::none,
                                        boost::optional<Eigen::MatrixXd&> H1 = boost::none,
                                        boost::optional<Eigen::MatrixXd&> H2 = boost::none,
                                        boost::optional<Eigen::MatrixXd&> H3 = boost::none,
                                        boost::optional<Eigen::MatrixXd&> H4 = boost::none) const override
  {
    auto error = compute_error(x0, x1, x2, x3, x4);
    if (H0)
    {
      derivative_x0.model = [&](const X0& _x0) { return compute_error(_x0, x1, x2, x3, x4); };
      *H0 = derivative_x0(x0);
    }

    if (H1)
    {
      derivative_x1.model = [&](const X1& _x1) { return compute_error(x0, _x1, x2, x3, x4); };
      *H1 = derivative_x1(x1);
    }

    if (H2)
    {
      derivative_x2.model = [&](const X2& _x2) { return compute_error(x0, x1, _x2, x3, x4); };
      *H2 = derivative_x2(x2);
    }

    if (H3)
    {
      derivative_x3.model = [&](const X3& _x3) { return compute_error(x0, x1, x2, _x3, x4); };
      *H3 = derivative_x3(x3);
    }

    if (H4)
    {
      derivative_x4.model = [&](const X4& _x4) { return compute_error(x0, x1, x2, x3, _x4); };
      *H4 = derivative_x4(x4);
    }

    return error;
  }

  virtual X0 compute_error(const X0& x0, const X1& x1, const X2& x2, const X3& x3, const X4& x4) const = 0;

private:
  partial_X0 partial_x0;
  partial_X1 partial_x1;
  partial_X2 partial_x2;
  partial_X3 partial_x3;
  partial_X4 partial_x4;

  mutable prx::math::first_order_derivative_t<partial_X0, X0, Evaluations> derivative_x0;
  mutable prx::math::first_order_derivative_t<partial_X1, X1, Evaluations> derivative_x1;
  mutable prx::math::first_order_derivative_t<partial_X2, X2, Evaluations> derivative_x2;
  mutable prx::math::first_order_derivative_t<partial_X3, X3, Evaluations> derivative_x3;
  mutable prx::math::first_order_derivative_t<partial_X4, X4, Evaluations> derivative_x4;
};

template <Eigen::Index Dim_X0, Eigen::Index Dim_X1, Eigen::Index Dim_X2, Eigen::Index Dim_X3, Eigen::Index Dim_X4,
          Eigen::Index Dim_X5, Evals Evaluations = 4>
class noise_model_6factor_t
  : public gtsam::NoiseModelFactor6<Eigen::Vector<double, Dim_X0>, Eigen::Vector<double, Dim_X1>,
                                    Eigen::Vector<double, Dim_X2>, Eigen::Vector<double, Dim_X3>,
                                    Eigen::Vector<double, Dim_X4>, Eigen::Vector<double, Dim_X5>>
{
protected:
  using X0 = Eigen::Vector<double, Dim_X0>;
  using X1 = Eigen::Vector<double, Dim_X1>;
  using X2 = Eigen::Vector<double, Dim_X2>;
  using X3 = Eigen::Vector<double, Dim_X3>;
  using X4 = Eigen::Vector<double, Dim_X4>;
  using X5 = Eigen::Vector<double, Dim_X5>;
  using partial_X0 = std::function<Eigen::Vector<double, Dim_X0>(const X0&)>;
  using partial_X1 = std::function<Eigen::Vector<double, Dim_X0>(const X1&)>;
  using partial_X2 = std::function<Eigen::Vector<double, Dim_X0>(const X2&)>;
  using partial_X3 = std::function<Eigen::Vector<double, Dim_X0>(const X3&)>;
  using partial_X4 = std::function<Eigen::Vector<double, Dim_X0>(const X4&)>;
  using partial_X5 = std::function<Eigen::Vector<double, Dim_X0>(const X5&)>;
  using Base = gtsam::NoiseModelFactor6<X0, X1, X2, X3, X4, X5>;

public:
  noise_model_6factor_t(gtsam::Key key_x0, gtsam::Key key_x1, gtsam::Key key_x2, gtsam::Key key_x3, gtsam::Key key_x4,
                        gtsam::Key key_x5, const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(cost_model, key_x0, key_x1, key_x2, key_x3, key_x4, key_x5)
    , derivative_x0(partial_x0, 0.01)
    , derivative_x1(partial_x1, 0.01)
    , derivative_x2(partial_x2, 0.01)
    , derivative_x3(partial_x3, 0.01)
    , derivative_x4(partial_x4, 0.01)
    , derivative_x5(partial_x5, 0.01)
  {
  }

  virtual ~noise_model_6factor_t()
  {
  }

  virtual Eigen::VectorXd evaluateError(const X0& x0, const X1& x1, const X2& x2, const X3& x3, const X4& x4,
                                        const X5& x5,  // no-lint
                                        boost::optional<Eigen::MatrixXd&> H0 = boost::none,
                                        boost::optional<Eigen::MatrixXd&> H1 = boost::none,
                                        boost::optional<Eigen::MatrixXd&> H2 = boost::none,
                                        boost::optional<Eigen::MatrixXd&> H3 = boost::none,
                                        boost::optional<Eigen::MatrixXd&> H4 = boost::none,
                                        boost::optional<Eigen::MatrixXd&> H5 = boost::none) const override
  {
    auto error = compute_error(x0, x1, x2, x3, x4, x5);
    if (H0)
    {
      derivative_x0.model = [&](const X0& _x0) { return compute_error(_x0, x1, x2, x3, x4, x5); };
      *H0 = derivative_x0(x0);
    }

    if (H1)
    {
      derivative_x1.model = [&](const X1& _x1) { return compute_error(x0, _x1, x2, x3, x4, x5); };
      *H1 = derivative_x1(x1);
    }

    if (H2)
    {
      derivative_x2.model = [&](const X2& _x2) { return compute_error(x0, x1, _x2, x3, x4, x5); };
      *H2 = derivative_x2(x2);
    }

    if (H3)
    {
      derivative_x3.model = [&](const X3& _x3) { return compute_error(x0, x1, x2, _x3, x4, x5); };
      *H3 = derivative_x3(x3);
    }

    if (H4)
    {
      derivative_x4.model = [&](const X4& _x4) { return compute_error(x0, x1, x2, x3, _x4, x5); };
      *H4 = derivative_x4(x4);
    }

    if (H5)
    {
      derivative_x5.model = [&](const X5& _x5) { return compute_error(x0, x1, x2, x3, x4, _x5); };
      *H5 = derivative_x5(x5);
    }

    return error;
  }

  virtual X0 compute_error(const X0& x0, const X1& x1, const X2& x2, const X3& x3, const X4& x4,
                           const X5& x5) const = 0;

private:
  partial_X0 partial_x0;
  partial_X1 partial_x1;
  partial_X2 partial_x2;
  partial_X3 partial_x3;
  partial_X4 partial_x4;
  partial_X5 partial_x5;

  mutable prx::math::first_order_derivative_t<partial_X0, X0, Evaluations> derivative_x0;
  mutable prx::math::first_order_derivative_t<partial_X1, X1, Evaluations> derivative_x1;
  mutable prx::math::first_order_derivative_t<partial_X2, X2, Evaluations> derivative_x2;
  mutable prx::math::first_order_derivative_t<partial_X3, X3, Evaluations> derivative_x3;
  mutable prx::math::first_order_derivative_t<partial_X4, X4, Evaluations> derivative_x4;
  mutable prx::math::first_order_derivative_t<partial_X5, X5, Evaluations> derivative_x5;
};
}  // namespace fg
}  // namespace prx