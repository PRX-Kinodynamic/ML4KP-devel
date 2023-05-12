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
template <Eigen::Index THETA_DIM>
class parameter_fusion_factor_t : public gtsam::NoiseModelFactor
{
  using derivative_ff = std::function<Eigen::Vector<double, THETA_DIM>(const Eigen::Vector<double, THETA_DIM>&)>;
  using theta_t = Eigen::Vector<double, THETA_DIM>;
  using weight_t = Eigen::Vector<double, THETA_DIM>;
  // using Base = gtsam::NoiseModelFactor5<value_t, value_t, value_t, value_t, value_t>;

public:
  // Container of keys.
  template <typename Container>
  parameter_fusion_factor_t(const gtsam::noiseModel::Base::shared_ptr& cost_model, Container theta_keys,
                            Container weight_keys, gtsam::Key key_theta_t)
    : gtsam::NoiseModelFactor(cost_model, merge_container<std::vector<gtsam::Key>>(theta_keys, weight_keys))
    , _key_theta_t(key_theta_t)
    , _theta_keys(theta_keys)
    , _weight_keys(weight_keys)
    , derivative_thp(0.01)
    , derivative_thetas(0.01)
    , derivative_weights(0.01)
  {
    keys_.push_back(key_theta_t);
  }

  virtual ~parameter_fusion_factor_t()
  {
  }

  Eigen::VectorXd unwhitenedError(const gtsam::Values& values,
                                  boost::optional<std::vector<Eigen::MatrixXd>&> H = boost::none) const override
  {
    Eigen::VectorXd error{ Eigen::VectorXd::Zero(THETA_DIM) };
    if (this->active(values))
    {
      gtsam::Key key_th, key_w;
      std::vector<theta_t> theta_values;
      std::vector<weight_t> weight_values;
      const theta_t th_t{ values.at<theta_t>(_key_theta_t) };
      for (std::size_t i = 0; i < _theta_keys.size(); ++i)
      {
        std::tie(key_th, key_w) = std::make_tuple(_theta_keys[i], _weight_keys[i]);
        theta_values.push_back(values.at<theta_t>(key_th));
        weight_values.push_back(values.at<weight_t>(key_w));
      }
      error = compute_error(th_t, theta_values, weight_values);
      if (H)
      {
        derivative_thp._model = [&](const theta_t& th_tp) { return compute_error(th_tp, theta_values, weight_values); };
        (*H)[0] = derivative_thp(theta_values[0]);

        const std::size_t half_hs{ ((*H).size() - 1) / 2 };
        for (int i = 1; i < half_hs; ++i)
        {
          derivative_thetas._model = [&](const theta_t& th_change) {
            std::vector<theta_t> tv{ theta_values };
            tv[i] = th_change;
            return compute_error(th_t, tv, weight_values);
          };
          (*H)[i] = derivative_thetas(theta_values[i]);

          derivative_weights._model = [&](const weight_t& w_change) {
            std::vector<weight_t> wv{ weight_values };
            wv[i] = w_change;
            return compute_error(th_t, theta_values, wv);
          };
          (*H)[i + half_hs] = derivative_weights(weight_values[i]);
        }
      }
    }
    return error;
  }

  Eigen::VectorXd compute_error(const theta_t& th_t, const std::vector<theta_t> thetas,
                                const std::vector<weight_t> weights) const
  {
    theta_t th_i;
    weight_t w_i;
    theta_t th_pt{ theta_t::Zero() };
    // for (auto th_w : prx::zip_iters(thetas, weights))
    for (std::size_t i = 0; i < thetas.size(); ++i)
    {
      std::tie(th_i, w_i) = std::make_tuple(thetas[i], weights[i]);
      // std::tie(th_i, w_i) = prx::unzip(th_w);
      th_pt += th_i.cwiseProduct(w_i);
    }
    return th_t - th_pt;
  }

private:
  gtsam::Key _key_theta_t;
  std::vector<gtsam::Key> _theta_keys;
  std::vector<gtsam::Key> _weight_keys;

  mutable math::first_order_derivative_t<derivative_ff, theta_t, 4> derivative_thp;
  mutable math::first_order_derivative_t<derivative_ff, theta_t, 4> derivative_thetas;
  mutable math::first_order_derivative_t<derivative_ff, weight_t, 4> derivative_weights;
};

}  // namespace fg
}  // namespace prx