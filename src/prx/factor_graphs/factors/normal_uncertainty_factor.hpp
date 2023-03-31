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
template <Eigen::Index DIM_0>
class normal_uncertainty_factor_t : public gtsam::NoiseModelFactor1<Eigen::Vector<double, DIM_0>>
{
  using Base = gtsam::NoiseModelFactor1<Eigen::Vector<double, DIM_0>>;

  using _covariance_t = Eigen::Matrix<double, DIM_0, DIM_0>;
  using _vector_t = Eigen::Vector<double, DIM_0>;

public:
  normal_uncertainty_factor_t(const gtsam::Key& key, const _Vector& goal, const _Matrix cost, const double sigma = 1e0)
    : Base(gtsam::noiseModel::Isotropic::Sigma(1, sigma), key), _cost(cost), _goal(goal)
  {
  }
  virtual Eigen::VectorXd evaluateError(const _Vector& x0,
                                        boost::optional<Eigen::MatrixXd&> H0 = boost::none) const override
  {
    const _Vector Xp{ x0 - _goal };
    const Eigen::VectorXd error{ Xp.transpose() * _cost * Xp };
    if (H0)
    {
      *H0 = x0.transpose() * (_cost + _cost.transpose());
    }
    return error;
  }

private:
  // _Matrix _A;
  _Matrix _cost;
  _Vector _goal;
};
}  // namespace fg
}  // namespace prx