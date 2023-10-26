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
template <Eigen::Index DimDist, Eigen::Index Dim0 = DimDist, Eigen::Index Dim1 = DimDist>
class euclidean_distance_factor_t
  : public gtsam::NoiseModelFactor2<Eigen::Vector<double, Dim0>, Eigen::Vector<double, Dim1>>
{
  using Base = gtsam::NoiseModelFactor2<Eigen::Vector<double, Dim0>, Eigen::Vector<double, Dim1>>;

public:
  using StateDist = Eigen::Vector<double, DimDist>;
  using State0 = Eigen::Vector<double, Dim0>;
  using State1 = Eigen::Vector<double, Dim1>;
  using MapFunction0 = std::function<StateDist(const State0&)>;
  using MapFunction1 = std::function<StateDist(const State1&)>;

  euclidean_distance_factor_t(double distance, gtsam::Key x1_key, gtsam::Key x2_key,
                              const gtsam::noiseModel::Base::shared_ptr& cost_model, const MapFunction0& f0,
                              const MapFunction1& f1)
    : Base(cost_model, x1_key, x2_key), _distance(distance), _map_func_0(f0), _map_func_1(f1)
  {
  }
  euclidean_distance_factor_t(double distance, gtsam::Key x1_key, gtsam::Key x2_key,
                              const gtsam::noiseModel::Base::shared_ptr& cost_model)
    // : Base(cost_model, x1_key, x2_key), _distance(distance)
    : euclidean_distance_factor_t(
          distance, x1_key, x2_key, cost_model, [](const State0& x0) { return x0; },
          [](const State1& x1) { return x1; })
  {
  }

  // error = _distance - norm2(x0 - x1)
  virtual Eigen::VectorXd evaluateError(const State0& x0, const State1& x1,  // no-lint
                                        boost::optional<Eigen::MatrixXd&> H0 = boost::none,
                                        boost::optional<Eigen::MatrixXd&> H1 = boost::none) const override
  {
    const StateDist difference{ _map_func_0(x0) - _map_func_1(x1) };
    // const StateDist difference{ StateDist::Ones() };
    const double norm2{ difference.norm() };
    const Eigen::Vector<double, 1> error{ _distance - norm2 };
    // const Eigen::Vector<double, 1> error{ 0 };

    if (H0)
    {
      *H0 = Eigen::Matrix<double, 1, Dim0>::Zero();
      (*H0).topLeftCorner<1, DimDist>() = -1.0 * difference.normalized().transpose();
    }

    if (H1)
    {
      // *H1 = +1.0 * difference.normalized().transpose();
      *H1 = Eigen::Matrix<double, 1, Dim1>::Zero();
      (*H1).topLeftCorner<1, DimDist>() = +1.0 * difference.normalized().transpose();
    }
    // PRX_DEBUG_VAR_1(_distance);
    // PRX_DEBUG_VAR_1(_map_func_0(x0).transpose());
    // PRX_DEBUG_VAR_1(_map_func_1(x1).transpose());
    // PRX_DEBUG_VAR_2(difference.transpose(), norm2);
    // PRX_DEBUG_VAR_1(error.transpose());
    // PRX_DEBUG_VAR_1((-1.0 * difference.normalized()).transpose());
    // PRX_DEBUG_VAR_1((+1.0 * difference.normalized()).transpose());
    return error;
  }

private:
  const double _distance;

  const MapFunction0 _map_func_0;
  const MapFunction1 _map_func_1;
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