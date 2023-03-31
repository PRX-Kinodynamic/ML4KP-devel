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
 * @brief
 *
 * @tparam     DIM_0  Dimension of x0
 * @tparam     DIM_1  Dimension of x1
 */
template <Eigen::Index Dim>
class state_prior_factor_t : public noise_model_1factor_t<Dim>
{
  using Base = noise_model_1factor_t<Dim>;

public:
  using State = typename Base::X0;

  template <typename T>
  state_prior_factor_t(gtsam::Key x0_key, const T& value, const space_t* ss,
                       const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(x0_key, cost_model), _ss(ss)
  {
    _prior = ss->make_point();
    _result = ss->make_point();
    _ss->copy(_prior, value);
  }

  virtual State compute_error(const State& state) const override
  {
    _ss->copy(_result, state);
    _ss->difference(_prior, _result, _result);
    return _result->vector<State>();
  }

private:
  const space_t* _ss;
  prx::space_point_t _prior;
  prx::space_point_t _result;
};

}  // namespace fg
}  // namespace prx