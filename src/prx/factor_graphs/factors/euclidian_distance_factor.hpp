#pragma once
#include <array>
#include <numeric>
#include <gtsam/config.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "prx/utilities/math/math_functions.hpp"
#include "prx/utilities/math/first_order_derivative.hpp"

#include "prx/factor_graphs/factors/noise_model_factor.hpp"

namespace prx
{
namespace fg
{

/**
 * @brief      This class describes an euclidian distance factor t.
 *
 * @tparam     Dim   { Dimension of the vector }
 */
template <Eigen::Index Dim>
class euclidian_distance_factor_t : public noise_model_3factor_t<1, Dim, Dim>
{
  using Base = noise_model_3factor_t<1, Dim, Dim>;

public:
  using Distance = typename Base::X0;
  using State = typename Base::X1;

  euclidian_distance_factor_t(
      double distance, gtsam::Key x1_key, gtsam::Key x2_key, gtsam::Values& values,
      const gtsam::noiseModel::Base::shared_ptr& cost_model = gtsam::noiseModel::Constrained::All(1))
    : Base(symbol_factory_t::create_hashed_symbol("fix_distance", x1_key, x2_key), x1_key, x2_key, cost_model)
  {
    values.insert_or_assign(symbol_factory_t::create_hashed_symbol("fix_distance", x1_key, x2_key), Distance(distance));
  }

  euclidian_distance_factor_t(gtsam::Key x0_key, gtsam::Key x1_key, gtsam::Key x2_key,
                              const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(x0_key, x1_key, x2_key, cost_model)
  {
  }

  virtual Distance compute_error(const Distance& distance, const State& x0, const State& x1) const override
  {
    // PRX_DEBUG_VAR_3(distance, x0.transpose(), x1.transpose());
    // PRX_DEBUG_VAR_2(distance, Distance((x0 - x1).norm()));
    return distance - Distance((x0 - x1).norm());
  }

private:
};

// Factor to enforce: Distance = norm2(x0 - x1)
template <Eigen::Index Dim>
class euclidean_distance_factor_t
  : public gtsam::NoiseModelFactor2<Eigen::Vector<double, Dim>, Eigen::Vector<double, Dim>>
{
  using Base = gtsam::NoiseModelFactor2<Eigen::Vector<double, Dim>, Eigen::Vector<double, Dim>>;

public:
  using State = Eigen::Vector<double, Dim>;

  euclidean_distance_factor_t(double distance, gtsam::Key x1_key, gtsam::Key x2_key,
                              const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(cost_model, x1_key, x2_key), _distance(distance)
  {
  }

  // error = _distance - norm2(x0 - x1)
  virtual Eigen::VectorXd evaluateError(const State& x0, const State& x1,  // no-lint
                                        boost::optional<Eigen::MatrixXd&> H0 = boost::none,
                                        boost::optional<Eigen::MatrixXd&> H1 = boost::none) const override
  {
    const State difference{ x0 - x1 };
    const double norm2{ difference.norm() };
    const Eigen::VectorXd error{ (Eigen::VectorXd(1) << _distance - norm2).finished() };

    if (H0)
    {
      *H0 = -1.0 * difference.normalized().transpose();
      // *H0 = (-1.0 * (difference / norm2_nz)).transpose();
    }

    if (H1)
    {
      *H1 = +1.0 * difference.normalized().transpose();
      // *H1 = (+1.0 * (difference / norm2_nz)).transpose();
    }
    // PRX_DEBUG_VAR_1(_distance);
    // PRX_DEBUG_VAR_1(x0.transpose());
    // PRX_DEBUG_VAR_1(x1.transpose());
    // PRX_DEBUG_VAR_2(difference.transpose(), norm2);
    // PRX_DEBUG_VAR_1(error.transpose());
    // PRX_DEBUG_VAR_1((-1.0 * difference.normalized()).transpose());
    // PRX_DEBUG_VAR_1((+1.0 * difference.normalized()).transpose());
    return error;
  }

  PRX_FACTOR_OVERLOAD_PRINT("euclidean_distance_factor_t")

private:
  const double _distance;
};

template <Eigen::Index Dim>
class normalize_factor_t : public gtsam::NoiseModelFactor1<Eigen::Vector<double, Dim>>
{
  using Base = gtsam::NoiseModelFactor1<Eigen::Vector<double, Dim>>;

public:
  using State = Eigen::Vector<double, Dim>;

  normalize_factor_t(gtsam::Key x1_key,
                     const gtsam::noiseModel::Base::shared_ptr& cost_model = gtsam::noiseModel::Constrained::All(1))
    : Base(cost_model, x1_key)
  {
  }

  virtual Eigen::VectorXd evaluateError(const State& x0,  // no-lint
                                        boost::optional<Eigen::MatrixXd&> H0 = boost::none) const override
  {
    const double norm2{ x0.norm() };
    const Eigen::VectorXd error{ (Eigen::VectorXd(1) << 1.0 - norm2).finished() };

    if (H0)
    {
      *H0 = -1.0 * x0.normalized().transpose();
      // *H0 = (-1.0 * (x0 / norm2)).transpose();
    }

    return error;
  }
};
}  // namespace fg
}  // namespace prx