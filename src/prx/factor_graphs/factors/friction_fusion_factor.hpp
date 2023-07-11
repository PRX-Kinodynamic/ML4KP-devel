#pragma once
#include <algorithm>
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

// ThetaPositions(x, y) -> Theta_key; ThetaPositions returns the theta that corresponds to the bottom left corner of the
// cell
template <Eigen::Index THETA_DIM, Eigen::Index BASIS_DIM, Eigen::Index StateDim, typename FrictionGrid>
class friction_fusion_factor_t : public gtsam::NoiseModelFactor
{
  using derivative_ff = std::function<Eigen::Vector<double, THETA_DIM>(const Eigen::Vector<double, THETA_DIM>&)>;

  using Theta = Eigen::Vector<double, THETA_DIM>;
  using Weights = Eigen::Vector<double, BASIS_DIM>;
  using Guard = Eigen::Vector<double, BASIS_DIM>;
  using Friction = Eigen::Vector<double, BASIS_DIM>;
  using State = Eigen::Vector<double, StateDim>;

  using partial_theta_fn = std::function<Theta(const Theta&)>;
  // using partial_weight_fn = std::function<Theta(const Weights&)>;
  using partial_friction_fn = std::function<Theta(const Friction&)>;
  using partial_guard_fn = std::function<Theta(const Guard&)>;

public:
  friction_fusion_factor_t(const gtsam::noiseModel::Base::shared_ptr& cost_model, const gtsam::Key frictions,
                           const gtsam::Key theta, const gtsam::Key guard, const State state,
                           const Eigen::Index theta_dim, const Eigen::Index basis_dim, const Eigen::Index guard_dim,
                           const FrictionGrid* frictions_grid_ptr = nullptr, const double h = prx::simulation_step)
    : gtsam::NoiseModelFactor(cost_model)
    , _derivative_theta(h, theta_dim, theta_dim)
    // , _derivative_weights(h, basis_dim, theta_dim)
    , _derivative_friction(h, basis_dim, theta_dim)
    , _derivative_guard(h, guard_dim, theta_dim)
    , _theta_zero(Theta::Zero(theta_dim))
    , _use_guard(true)
    , _frictions_grid(frictions_grid_ptr)
    , _state(state)
  {
    // keys_.push_back(weights);
    keys_.push_back(frictions);
    keys_.push_back(theta);
    keys_.push_back(guard);
  }
  friction_fusion_factor_t(const gtsam::noiseModel::Base::shared_ptr& cost_model, const gtsam::Key frictions,
                           const gtsam::Key theta, const Eigen::Index theta_dim, const Eigen::Index basis_dim,
                           const double h = prx::simulation_step)
    : gtsam::NoiseModelFactor(cost_model)
    , _derivative_theta(h, theta_dim, theta_dim)
    // , _derivative_weights(h, basis_dim, theta_dim)
    , _derivative_friction(h, basis_dim, theta_dim)
    , _theta_zero(Theta::Zero(theta_dim))
    , _use_guard(false)
  {
    // keys_.push_back(weights);
    keys_.push_back(frictions);
    keys_.push_back(theta);
  }

  template <Eigen::Index ThDim = THETA_DIM, std::enable_if_t<(ThDim != Eigen::Dynamic), bool> = true>
  friction_fusion_factor_t(const gtsam::noiseModel::Base::shared_ptr& cost_model, const gtsam::Key frictions,
                           const gtsam::Key theta, const gtsam::Key guard, const State state,
                           const FrictionGrid* frictions_grid_ptr = nullptr, const double h = prx::simulation_step)
    : friction_fusion_factor_t(cost_model, frictions, theta, guard, state, THETA_DIM, BASIS_DIM, BASIS_DIM,
                               frictions_grid_ptr, h)
  {
  }

  template <Eigen::Index ThDim = THETA_DIM, std::enable_if_t<(ThDim != Eigen::Dynamic), bool> = true>
  friction_fusion_factor_t(const gtsam::noiseModel::Base::shared_ptr& cost_model, const gtsam::Key frictions,
                           const gtsam::Key theta, const double h = prx::simulation_step)
    : friction_fusion_factor_t(cost_model, frictions, theta, THETA_DIM, BASIS_DIM, h)
  {
  }

  virtual ~friction_fusion_factor_t()
  {
  }

  Eigen::VectorXd unwhitenedError(const gtsam::Values& values,
                                  boost::optional<std::vector<Eigen::MatrixXd>&> H = boost::none) const override
  {
    Eigen::VectorXd error{ _theta_zero };
    if (this->active(values))
    {
      // const Weights weights = values.at<Weights>(keys_[0]);
      const Friction friction = values.at<Friction>(keys_[0]);
      const Theta theta = values.at<Theta>(keys_[1]);
      const Weights weights{ compute_weight(_state) };
      Guard guard;
      if (_use_guard)
      {
        guard = values.at<Guard>(keys_[2]);
        error = compute_error(theta, friction, guard, weights);
      }
      else
      {
        error = compute_error(theta, friction, weights);
      }
      if (H)
      {
        if (_use_guard)
        {
          // _derivative_weights._model = [&](const Weights& w_) { return compute_error(theta, w_, friction, guard); };
          _derivative_theta._model = [&](const Theta& th_) { return compute_error(th_, friction, guard, weights); };
          _derivative_friction._model = [&](const Friction& f_) { return compute_error(theta, f_, guard, weights); };
          _derivative_guard._model = [&](const Guard& g_) { return compute_error(theta, friction, g_, weights); };
        }
        else
        {
          _derivative_theta._model = [&](const Theta& th_) { return compute_error(th_, friction, weights); };
          _derivative_friction._model = [&](const Friction& f_) { return compute_error(theta, f_, weights); };
          // _derivative_weights._model = [&](const Weights& w_) { return compute_error(theta, w_, friction); };
        }

        // (*H)[0] = _derivative_weights(weights);
        (*H)[0] = _derivative_friction(friction);
        (*H)[1] = _derivative_theta(theta);
        if (_use_guard)
          (*H)[2] = _derivative_guard(guard);
      }
      // PRX_DEBUG_VAR_1("----------------");
      // PRX_DEBUG_VAR_1(theta.transpose());
      // PRX_DEBUG_VAR_1(weights.transpose());
      // PRX_DEBUG_VAR_1(friction.transpose());
      // PRX_DEBUG_VAR_1(guard.transpose());
      // PRX_DEBUG_VAR_1(error.transpose());
    }
    return error;
  }

  Weights compute_weight(const State& state) const
  {
    Weights weights{ Weights::Zero() };
    const double length_0{ _frictions_grid->get_cell_length(0) };
    const double length_1{ _frictions_grid->get_cell_length(1) };
    const double _D(std::sqrt(length_0 * length_0 + length_1 * length_1));
    std::size_t i = 0;
    for (auto iter = _frictions_grid->begin(); iter != _frictions_grid->end(); iter++, i++)
    {
      Eigen::Vector2d position{ _frictions_grid->template unmap_key<Eigen::Vector2d>((*iter).first) };
      const Eigen::Vector2d delta = position - state.head(2);
      weights[i] = std::max(1.0 - (delta.lpNorm<1>() / _D), 0.0);
    }
    weights = weights / weights.sum();
    return weights;
  }

  Theta compute_error(const Theta& th_t, const Friction& thetas, const Weights& weights) const
  {
    Theta error{ _theta_zero };
    error[0] = th_t[0] - weights.dot(thetas);

    return error;
  }

  Theta compute_error(const Theta& th_t, const Friction& thetas, const Guard& guard, const Weights& weights) const
  {
    Theta error{ _theta_zero };
    const Weights guarded_weight{ weights.array() * guard.array() };
    error[0] = th_t[0] - guarded_weight.dot(thetas);

    // PRX_DEBUG_VAR_1("--------------");
    // PRX_DEBUG_VAR_1(guarded_weight.transpose());
    // PRX_DEBUG_VAR_1(thetas.transpose());
    // PRX_DEBUG_VAR_3(th_t[0], error[0], guarded_weight.dot(thetas));
    return error;
  }

private:
  mutable math::first_order_derivative_t<partial_theta_fn, Theta, 4> _derivative_theta;
  // mutable math::first_order_derivative_t<partial_weight_fn, Weights, 4> _derivative_weights;
  mutable math::first_order_derivative_t<partial_friction_fn, Friction, 4> _derivative_friction;
  mutable math::first_order_derivative_t<partial_guard_fn, Guard, 4> _derivative_guard;

  const bool _use_guard;
  const Theta _theta_zero;
  const FrictionGrid* _frictions_grid;
  const State _state;
};

template <typename Weights, typename StateIn, typename Positions>
Weights compute_weight(const StateIn& state, const double cell_size, const Positions& basis_positions)
{
  Weights weights{ Weights::Zero() };
  const double _D(std::sqrt(cell_size * cell_size + cell_size * cell_size));
  // PRX_DEBUG_VAR_2(_D, state.head(2).transpose());
  for (int i = 0; i < 4; ++i)
  {
    const Eigen::Vector2d position{ basis_positions.row(i) };
    const Eigen::Vector2d delta{ position - state.head(2) };
    // PRX_DEBUG_VAR_2(position.transpose(), delta.transpose());
    weights[i] = std::max(1.0 - (delta.lpNorm<1>() / _D), 0.0);
  }
  // PRX_DEBUG_VAR_1(weights.transpose());
  weights = weights / weights.sum();
  // PRX_DEBUG_VAR_1(weights.transpose());
  return weights;
}

template <Eigen::Index THETA_DIM, typename State, typename BasisPositions>
class friction_local_fusion_factor_t : public noise_model_5factor_t<THETA_DIM, 1, 1, 1, 1>
{
  using derivative_ff = std::function<Eigen::Vector<double, THETA_DIM>(const Eigen::Vector<double, THETA_DIM>&)>;

  static constexpr Eigen::Index BASIS_DIM{ 1 };
  using Theta = Eigen::Vector<double, THETA_DIM>;

  using Basis = Eigen::Vector<double, BASIS_DIM>;
  using Guard = Eigen::Vector<double, 4>;
  using Weights = Eigen::Vector<double, 4>;

  using partial_theta_fn = std::function<Theta(const Theta&)>;
  using partial_basis_fn = std::function<Theta(const Basis&)>;
  // using partial_weight_fn = std::function<Theta(const Weights&)>;
  // using partial_guard_fn = std::function<Theta(const Guard&)>;

  using Base = noise_model_5factor_t<THETA_DIM, 1, 1, 1, 1>;

public:
  friction_local_fusion_factor_t(const gtsam::noiseModel::Base::shared_ptr& cost_model, const gtsam::Key theta,
                                 const gtsam::Key basis_0, const gtsam::Key basis_1, const gtsam::Key basis_2,
                                 const gtsam::Key basis_3, const State state, const BasisPositions basis_positions,
                                 const double cell_size, const Eigen::Index theta_dim, const Eigen::Index basis_dim,
                                 const double h = prx::simulation_step)
    : Base(theta, basis_0, basis_1, basis_2, basis_3, cost_model)
    , _derivative_theta(h, theta_dim, theta_dim)
    , _derivative_basis(h, basis_dim, theta_dim)
    , _theta_zero(Theta::Zero(theta_dim))
    , _state(state)


  {
    // const Weights weights{ compute_weight(_state) };
    // _guarded_weight = weights.array() * guard.array();
  }

  template <Eigen::Index ThDim = THETA_DIM, std::enable_if_t<(ThDim != Eigen::Dynamic), bool> = true>
  friction_local_fusion_factor_t(const gtsam::noiseModel::Base::shared_ptr& cost_model, const gtsam::Key theta,
                                 const gtsam::Key basis_0, const gtsam::Key basis_1, const gtsam::Key basis_2,
                                 const gtsam::Key basis_3, const State state, const BasisPositions positions,
                                 const double cell_size, const double h = prx::simulation_step)
    : friction_local_fusion_factor_t(cost_model, theta, basis_0, basis_1, basis_2, basis_3, state, cell_size, THETA_DIM,
                                     BASIS_DIM, h)
  {
  }

  virtual ~friction_local_fusion_factor_t()
  {
  }


  Theta compute_error(const Theta& th_t, const Basis& basis_0, const Basis& basis_1, const Basis& basis_2,
                      const Basis& basis_3) const
  {
    Theta error{ _theta_zero };
    Weights basis(basis_0[0], basis_1[0], basis_2[0], basis_3[0]);
    error[0] = th_t[0] - _guarded_weight.dot(basis);

    // PRX_DEBUG_VAR_1("--------------");
    // PRX_DEBUG_VAR_1(guarded_weight.transpose());
    // PRX_DEBUG_VAR_1(basis.transpose());
    // PRX_DEBUG_VAR_1(_guarded_weight.transpose());
    // PRX_DEBUG_VAR_3(th_t[0], error[0], _guarded_weight.dot(basis));
    return error;
  }

private:
  mutable math::first_order_derivative_t<partial_theta_fn, Theta, 4> _derivative_theta;
  // mutable math::first_order_derivative_t<partial_weight_fn, Weights, 4> _derivative_weights;
  mutable math::first_order_derivative_t<partial_basis_fn, Basis, 4> _derivative_basis;
  // mutable math::first_order_derivative_t<partial_guard_fn, Guard, 4> _derivative_guard;


  const Theta _theta_zero;
  const State _state;
  const BasisPositions _basis_positions;
  const double _cell_size;
  const Weights _guarded_weight;
};

}  // namespace fg
}  // namespace prx
