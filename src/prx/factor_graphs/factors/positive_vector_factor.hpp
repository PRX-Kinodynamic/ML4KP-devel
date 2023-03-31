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

}  // namespace fg
}  // namespace prx