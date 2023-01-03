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

  // bool is_theta_relevant(const state_t theta_t_pos, const gtsam::Key& theta_key, state_t& theta_i_pos) const
  // {
  //   const double x_cell_length{ _theta_pos.get_cell_length(0) };
  //   const double y_cell_length{ _theta_pos.get_cell_length(1) };

  //   const double x0{ theta_t_pos[0] };
  //   const double y0{ theta_t_pos[1] };

  //   const double x1{ theta_t_pos[0] + x_cell_length };
  //   const double y1{ theta_t_pos[1] };

  //   const double x2{ theta_t_pos[0] };
  //   const double y2{ theta_t_pos[1] + y_cell_length };

  //   const double x3{ theta_t_pos[0] + x_cell_length };
  //   const double y3{ theta_t_pos[1] + y_cell_length };

  //   if (_theta_pos.in_bounds(x0, y0) && _theta_pos(x0, y0) == theta_key)
  //   {
  //     theta_i_pos = _theta_pos.template unmap<state_t>(x0, y0);
  //     return true;
  //   }
  //   if (_theta_pos.in_bounds(x1, y1) && _theta_pos(x1, y1) == theta_key)
  //   {
  //     theta_i_pos = _theta_pos.template unmap<state_t>(x1, y1);
  //     return true;
  //   }
  //   if (_theta_pos.in_bounds(x2, y2) && _theta_pos(x2, y2) == theta_key)
  //   {
  //     theta_i_pos = _theta_pos.template unmap<state_t>(x2, y2);
  //     return true;
  //   }
  //   if (_theta_pos.in_bounds(x3, y3) && _theta_pos(x3, y3) == theta_key)
  //   {
  //     theta_i_pos = _theta_pos.template unmap<state_t>(x3, y3);
  //     return true;
  //   }
  //   return false;
  // }

  // double compute_weight(const state_t theta_t_pos, const state_t& theta_i_pos) const
  // {
  //   const double Bx{ theta_i_pos[0] };
  //   const double By{ theta_i_pos[1] };

  //   const double Xx{ theta_t_pos[0] };
  //   const double Xy{ theta_t_pos[1] };

  //   const double x_cell_length{ _theta_pos.get_cell_length(0) };
  //   const double y_cell_length{ _theta_pos.get_cell_length(1) };

  //   const double delta_x{ std::pow(Bx - Xx, 2) };
  //   const double delta_y{ std::pow(By - Xy, 2) };

  //   if (delta_x == 0 && delta_y == 0)
  //     return 10000;
  //   const double D{ std::sqrt(delta_x + delta_y) };

  //   return 1 / (D);
  // }

  theta_t compute_error(const theta_t& th_t, const weights_t weights, const frictions_t thetas) const
  {
    // std::cout << "th_t: " << th_t[0] << std::endl;
    // std::cout << "ws: " << weights.transpose() << std::endl;
    // std::cout << "thetas: " << thetas.transpose() << std::endl;
    // std::cout << "dot: " << weights.dot(thetas) << std::endl;
    // PRX_DEBUG_VAR_2(th_t, weights.dot(thetas));
    const theta_t error{ th_t[0] - weights.dot(thetas) };
    // const theta_t error{ weights.dot(thetas) - th_t[0] };
    // std::cout << "error: " << error << std::endl;
    return error;
    // return th_t -
    //        weights.adjoint() * thetas;  // equivalent to adjoint but returns a 1x1 Mat (1-vec) instead of a double
  }

private:
  mutable math::first_order_derivative_t<partial_theta_fn, theta_t, 4> _derivative_theta;
  mutable math::first_order_derivative_t<partial_weight_fn, weights_t, 4> _derivative_weights;
  mutable math::first_order_derivative_t<partial_frictions_fn, frictions_t, 4> _derivative_frictions;
};
}  // namespace fg
}  // namespace prx