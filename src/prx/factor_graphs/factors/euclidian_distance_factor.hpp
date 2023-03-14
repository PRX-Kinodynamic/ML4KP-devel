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

}  // namespace fg
}  // namespace prx