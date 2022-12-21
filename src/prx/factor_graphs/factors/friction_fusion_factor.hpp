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
// ThetaPositions(x, y) -> Theta_key; ThetaPositions returns the theta that corresponds to the bottom left corner of the
// cell
template <Eigen::Index X_DIM, Eigen::Index THETA_DIM, Eigen::Index Friction_DIM, class ThetaPositions>
class friction_fusion_factor_t : public gtsam::NoiseModelFactor
{
  using derivative_ff = std::function<Eigen::Vector<double, THETA_DIM>(const Eigen::Vector<double, THETA_DIM>&)>;
  using theta_t = Eigen::Vector<double, THETA_DIM>;
  using weight_t = Eigen::Vector<double, THETA_DIM>;
  using state_t = Eigen::Vector<double, X_DIM>;
  using friction_t = Eigen::Vector<double, Friction_DIM>;
  using partial_fn = std::function<theta_t(const friction_t&)>;

public:
  template <typename Container>
  friction_fusion_factor_t(const gtsam::noiseModel::Base::shared_ptr& cost_model, Container thetas_keys,
                           gtsam::Key theta_t_key, gtsam::Key x_key, const ThetaPositions& theta_pos)
    : gtsam::NoiseModelFactor(cost_model, thetas_keys)
    // , _theta_keys(theta_keys)
    , _derivative(0.01)
    // , derivative_thetas(0.01)
    // , derivative_weights(0.01)
    , _theta_pos(theta_pos)
    , _theta_t_key(theta_t_key)
    , _x_key(x_key)
  {
    // keys_.push_back(theta_t_key);
    // keys_.push_back(x_key);
  }

  virtual ~friction_fusion_factor_t()
  {
  }

  Eigen::VectorXd unwhitenedError(const gtsam::Values& values,
                                  boost::optional<std::vector<Eigen::MatrixXd>&> H = boost::none) const override
  {
    // PRX_DEBUG_PRINT;
    const std::size_t total_thetas{ keys_.size() };
    const theta_t theta_t_val{ values.at<theta_t>(_theta_t_key) };

    friction_t th{ friction_t::Zero() };
    friction_t weights{ friction_t::Zero() };

    state_t theta_i_pos{ state_t::Zero() };
    const state_t theta_t_pos{ values.at<state_t>(_x_key) };
    for (int i = 0; i < total_thetas; ++i)
    {
      th[i] = values.at<theta_t>(keys_[i])[0];  // Assuming THETA_DIM==1 for now.

      if (is_theta_relevant(theta_t_pos, keys_[i], theta_i_pos))
      {
        weights[i] = compute_weight(theta_t_pos, theta_i_pos);
      }
    }

    // PRX_DEBUG_PRINT;
    const Eigen::VectorXd error{ compute_error(theta_t_val, weights, th) };
    if (H)
    {
      for (int i = 0; i < total_thetas; ++i)
      {
        _derivative.model = [&](const friction_t& th_) { return compute_error(theta_t_val, weights, th_); };
        auto dev = _derivative(th);

        // std::cout << "dev: " << dev << std::endl;
        (*H)[i] = dev;
        // (*H)[i] = _derivative(th);
      }
    }
    // PRX_DEBUG_PRINT;
    return error;
  }

  bool is_theta_relevant(const state_t theta_t_pos, const gtsam::Key& theta_key, state_t& theta_i_pos) const
  {
    const double x_cell_length{ _theta_pos.get_cell_length(0) };
    const double y_cell_length{ _theta_pos.get_cell_length(1) };

    const double x0{ theta_t_pos[0] };
    const double y0{ theta_t_pos[1] };

    const double x1{ theta_t_pos[0] + x_cell_length };
    const double y1{ theta_t_pos[1] };

    const double x2{ theta_t_pos[0] };
    const double y2{ theta_t_pos[1] + y_cell_length };

    const double x3{ theta_t_pos[0] + x_cell_length };
    const double y3{ theta_t_pos[1] + y_cell_length };

    // PRX_DEBUG_VAR_1(prx::key_formatter(theta_key));
    // PRX_DEBUG_VAR_3(x0, y0, prx::key_formatter(_theta_pos(x0, y0)));
    // PRX_DEBUG_VAR_3(x1, y1, prx::key_formatter(_theta_pos(x1, y1)));
    // PRX_DEBUG_VAR_3(x2, y2, prx::key_formatter(_theta_pos(x2, y2)));
    // PRX_DEBUG_VAR_3(x3, y3, prx::key_formatter(_theta_pos(x3, y3)));
    if (_theta_pos.in_bounds(x0, y0) && _theta_pos(x0, y0) == theta_key)
    {
      theta_i_pos = _theta_pos.template unmap<state_t>(x0, y0);
      return true;
    }
    if (_theta_pos.in_bounds(x1, y1) && _theta_pos(x1, y1) == theta_key)
    {
      theta_i_pos = _theta_pos.template unmap<state_t>(x1, y1);
      return true;
    }
    if (_theta_pos.in_bounds(x2, y2) && _theta_pos(x2, y2) == theta_key)
    {
      theta_i_pos = _theta_pos.template unmap<state_t>(x2, y2);
      return true;
    }
    if (_theta_pos.in_bounds(x3, y3) && _theta_pos(x3, y3) == theta_key)
    {
      theta_i_pos = _theta_pos.template unmap<state_t>(x3, y3);
      return true;
    }
    return false;
  }

  double compute_weight(const state_t theta_t_pos, const state_t& theta_i_pos) const
  {
    const double Bx{ theta_i_pos[0] };
    const double By{ theta_i_pos[1] };

    const double Xx{ theta_t_pos[0] };
    const double Xy{ theta_t_pos[1] };

    const double x_cell_length{ _theta_pos.get_cell_length(0) };
    const double y_cell_length{ _theta_pos.get_cell_length(1) };

    const double delta_x{ std::fabs(Bx - Xx) };
    const double delta_y{ std::fabs(By - Xy) };

    const double D{ x_cell_length + y_cell_length };

    return 1 - (delta_x + delta_y) / (D);
  }

  theta_t compute_error(const theta_t& th_t, const friction_t weights, const friction_t thetas) const
  {
    // std::cout << "th_t: " << th_t[0] << std::endl;
    // std::cout << "ws: " << weights.transpose() << std::endl;
    // std::cout << "dot: " << weights.dot(thetas) << std::endl;
    const theta_t error{ th_t[0] - weights.dot(thetas) };
    // std::cout << "error: " << error << std::endl;
    return error;
    // return th_t -
    //        weights.adjoint() * thetas;  // equivalent to adjoint but returns a 1x1 Mat (1-vec) instead of a double
  }

private:
  // std::vector<gtsam::Key> _theta_keys;
  ThetaPositions _theta_pos;
  gtsam::Key _theta_t_key;
  gtsam::Key _x_key;
  mutable math::first_order_derivative_t<partial_fn, friction_t, 4> _derivative;
};
}  // namespace fg
}  // namespace prx