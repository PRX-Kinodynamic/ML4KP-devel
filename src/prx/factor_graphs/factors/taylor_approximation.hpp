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
// template <Eigen::Index InputDim, Eigen::Index OutputDim>
// class taylor_approximation_t : public gtsam::NoiseModelFactor
// {
//   using Input = Eigen::Vector<double, InputDim>;
//   using Output = Eigen::Vector<double, OutputDim>;
//   using PartialFn = std::function<Output(const Input&)>;

// public:
//   template <typename Container>
//   taylor_approximation_t(const gtsam::noiseModel::Base::shared_ptr& cost_model, const Container coefficient_keys,
//                          const Eigen::Index input_dim, const Eigen::Index output_dim,
//                          const double h = prx::simulation_step)
//     : gtsam::NoiseModelFactor(cost_model)
//   {
//     keys_.insert(keys_.end(), coefficient_keys.begin(), coefficient_keys.end());
//   }

//   template <typename Container, Eigen::Index StateDim = InputDim,
//             std::enable_if_t<(StateDim != Eigen::Dynamic), bool> = true>
//   taylor_approximation_t(const gtsam::noiseModel::Base::shared_ptr& cost_model, const Container coefficient_keys,
//                          const gtsam::Key weights, const gtsam::Key theta, const double h = prx::simulation_step)
//     : taylor_approximation_t(cost_model, coefficient_keys, InputDim, OutputDim, h)
//   {
//   }

//   virtual ~taylor_approximation_t()
//   {
//   }

//   Eigen::VectorXd unwhitenedError(const gtsam::Values& values,
//                                   boost::optional<std::vector<Eigen::MatrixXd>&> H = boost::none) const override
//   {
//     Eigen::VectorXd error{ _theta_zero };

//     if (this->active(values))
//     {
//       const weights_t weights_vec = values.at<weights_t>(keys_[0]);
//       const frictions_t friction_vec = values.at<frictions_t>(keys_[1]);
//       const theta_t theta = values.at<theta_t>(keys_[2]);
//       error = compute_error(theta, weights_vec, friction_vec);
//       if (H)
//       {
//         _derivative_theta._model = [&](const theta_t& th_) { return compute_error(th_, weights_vec, friction_vec); };
//         _derivative_weights._model = [&](const weights_t& w_) { return compute_error(theta, w_, friction_vec); };
//         _derivative_frictions._model = [&](const frictions_t& fr_) { return compute_error(theta, weights_vec, fr_);
//         };

//         const auto dev_th = _derivative_theta(theta);
//         const auto dev_we = _derivative_weights(weights_vec);
//         const auto dev_fr = _derivative_frictions(friction_vec);

//         (*H)[0] = dev_we;
//         (*H)[1] = dev_fr;
//         (*H)[2] = dev_th;
//       }
//     }
//     return error;
//   }

//   theta_t compute_error(const theta_t& th_t, const weights_t weights, const frictions_t thetas) const
//   {
//     error[0] = th_t[0] - weights.dot(thetas);

//     return error;
//   }

// private:
//   mutable math::first_order_derivative_t<partial_theta_fn, theta_t, 4> _derivative_theta;
//   mutable math::first_order_derivative_t<partial_weight_fn, weights_t, 4> _derivative_weights;
//   mutable math::first_order_derivative_t<partial_frictions_fn, frictions_t, 4> _derivative_frictions;

//   const theta_t _theta_zero;
// };
}  // namespace fg
}  // namespace prx