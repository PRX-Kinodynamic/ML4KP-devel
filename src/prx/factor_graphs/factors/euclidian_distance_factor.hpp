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
class euclidian_distance_factor_t : public noise_model_3factor_t<1, Dim, Dim>
{
  using Base = noise_model_3factor_t<1, Dim, Dim>;

public:
  using Distance = typename Base::X0;
  using State = typename Base::X1;

  euclidian_distance_factor_t(gtsam::Key x0_key, gtsam::Key x1_key, gtsam::Key x2_key,
                              const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(x0_key, x1_key, x2_key, cost_model)
  {
  }

  virtual Distance compute_error(const Distance& distance, const State& x0, const State& x1) const override
  {
    return distance - Distance((x0 - x1).norm());
  }

private:
};

}  // namespace fg
}  // namespace prx