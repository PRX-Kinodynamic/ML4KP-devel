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
template <Eigen::Index THETA_DIM, Eigen::Index BASIS_DIM>
class friction_fusion_factor_t : public gtsam::NoiseModelFactor
{
  using derivative_ff = std::function<Eigen::Vector<double, THETA_DIM>(const Eigen::Vector<double, THETA_DIM>&)>;
  using theta_t = Eigen::Vector<double, THETA_DIM>;
  using weights_t = Eigen::Vector<double, BASIS_DIM>;
  using frictions_t = Eigen::Vector<double, BASIS_DIM>;
  using partial_theta_fn = std::function<theta_t(const theta_t&)>;
  using partial_weight_fn = std::function<theta_t(const weights_t&)>;
  using partial_frictions_fn = std::function<theta_t(const frictions_t&)>;

public:
  friction_fusion_factor_t(const gtsam::noiseModel::Base::shared_ptr& cost_model, const gtsam::Key frictions,
                           const gtsam::Key weights, const gtsam::Key theta)
    : gtsam::NoiseModelFactor(cost_model)
    , _derivative_weights(0.01)
    , _derivative_frictions(0.01)
    , _derivative_theta(0.01)
  {
    keys_.push_back(weights);
    keys_.push_back(frictions);
    keys_.push_back(theta);
  }

  virtual ~friction_fusion_factor_t()
  {
  }

  Eigen::VectorXd unwhitenedError(const gtsam::Values& values,
                                  boost::optional<std::vector<Eigen::MatrixXd>&> H = boost::none) const override
  {
    Eigen::VectorXd error{ theta_t::Zero() };

    if (this->active(values))
    {
      const weights_t weights_vec = values.at<weights_t>(keys_[0]);
      const frictions_t friction_vec = values.at<frictions_t>(keys_[1]);
      const theta_t theta = values.at<theta_t>(keys_[2]);
      error = compute_error(theta, weights_vec, friction_vec);
      if (H)
      {
        _derivative_theta.model = [&](const theta_t& th_) { return compute_error(th_, weights_vec, friction_vec); };
        _derivative_weights.model = [&](const weights_t& w_) { return compute_error(theta, w_, friction_vec); };
        _derivative_frictions.model = [&](const frictions_t& fr_) { return compute_error(theta, weights_vec, fr_); };

        const auto dev_th = _derivative_theta(theta);
        const auto dev_we = _derivative_weights(weights_vec);
        const auto dev_fr = _derivative_frictions(friction_vec);

        (*H)[0] = dev_we;
        (*H)[1] = dev_fr;
        (*H)[2] = dev_th;
      }
    }
    return error;
  }

  theta_t compute_error(const theta_t& th_t, const weights_t weights, const frictions_t thetas) const
  {
    const theta_t error{ th_t[0] - weights.dot(thetas) };

    return error;
  }

private:
  mutable math::first_order_derivative_t<partial_theta_fn, theta_t, 4> _derivative_theta;
  mutable math::first_order_derivative_t<partial_weight_fn, weights_t, 4> _derivative_weights;
  mutable math::first_order_derivative_t<partial_frictions_fn, frictions_t, 4> _derivative_frictions;
};
}  // namespace fg
}  // namespace prx