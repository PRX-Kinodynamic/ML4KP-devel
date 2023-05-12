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
template <Eigen::Index BasisDim>
class basis_weights_factor_t : public noise_model_1factor_t<BasisDim>
{
  using Base = noise_model_1factor_t<Dim>;
  using NoiseModel = typename Base::NoiseModel;

public:
  using Basis = typename Base::X0;
  using State = typename Base::X1;

  template <typename State>
  basis_weights_factor_t(gtsam::Key basis_key, const State& state, const double length, const NoiseModel& cost_model,
                         Eigen::Index basis_dim, const double h = simulation_step)
    : Base(basis_key, cost_model, basis_dim, h) _D(std::sqrt(length * length + length * length))
  {
    _state = state;
  }
  template <typename State, Eigen::Index StateDim = Dim, std::enable_if_t<(StateDim != Eigen::Dynamic), bool> = true>
  basis_weights_factor_t(gtsam::Key basis_key, const State& state, const double length, const NoiseModel& cost_model,
                         const double h = simulation_step)
    : basis_weights_factor_t(basis_key, state, cost_model, BasisDim, h)
  {
  }

  template <typename BasisPosition>
  double compute_weight(const BasisPosition& basis_position, const State& state)
  {
    const auto delta = basis_position.head(2) - state.head(2);
    return std::max(1.0 - (delta.lpNorm<1>() / _D), 0.0);
  }

  virtual Basis compute_error(const Basis& basis, const State& state) const
  {
    BasisVector basis_p{ BasisVector::Zero() };
    // for (int i = 0; i < error.size(); ++i)
    std::size_t i = 0;
    for (auto pair_ : pos_grid)
    {
      basis_p[i] = compute_weight(basis, state);
    }
    return basis - basis_p;
  }

private:
  const space_t* _ss;
  prx::space_point_t _prior;
  prx::space_point_t _result;
};

}  // namespace fg
}  // namespace prx