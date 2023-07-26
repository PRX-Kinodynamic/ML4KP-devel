#pragma once
#include <array>
#include <numeric>
#include <gtsam/config.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "prx/factor_graphs/factors/noise_model_factor.hpp"

namespace prx
{
namespace fg
{

template <Eigen::Index Dim>
class positive_vector_factor_t : public noise_model_1factor_t<Dim>
{
  using Base = noise_model_1factor_t<Dim>;

public:
  using Vector = typename Base::X0;

  positive_vector_factor_t(const gtsam::Key& key, const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(key, cost_model), _zero(Vector::Zero().array())
  {
  }

  virtual Vector compute_error(const Vector& x) const override
  {
    const Eigen::Array<double, Dim, 1> x_array{ x.array() };
    const auto greater_than_zero = x_array < _zero;
    const Vector error{ x_array * greater_than_zero.template cast<double>() };
    // PRX_DEBUG_VAR_1(error)
    return error;
  }

private:
  Eigen::Array<double, Dim, 1> _zero;
};

// Force a positive value in one index of the vector.
// I.e. Given a vector (x,y,z), only force z>=0 while accepting {x,y} < 0. Using mask (0,0,1)
// A mask of Ones(Dim) is the same as positive_vector_factor_t
template <Eigen::Index Dim>
class partial_positive_vector_factor_t : public noise_model_1factor_t<Dim>
{
  using Base = noise_model_1factor_t<Dim>;

public:
  using Vector = typename Base::X0;

  partial_positive_vector_factor_t(const Vector mask, const gtsam::Key& key,
                                   const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(key, cost_model), _zero(Vector::Zero().array()), _mask(mask.array())
  {
  }

  virtual Vector compute_error(const Vector& x) const override
  {
    const Eigen::Array<double, Dim, 1> x_array{ x.array() * _mask };
    const auto greater_than_zero = x_array < _zero;
    const Vector error{ x_array * greater_than_zero.template cast<double>() };
    // PRX_DEBUG_VAR_1(error)
    return error;
  }

private:
  Eigen::Array<double, Dim, 1> _zero;
  Eigen::Array<double, Dim, 1> _mask;
};

}  // namespace fg
}  // namespace prx