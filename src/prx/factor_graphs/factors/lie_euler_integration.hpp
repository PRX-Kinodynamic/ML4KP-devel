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
template <typename State>
class lie_euler_integrator_factor_t : public gtsam::NoiseModelFactor
{
  using Base = gtsam::NoiseModelFactor;
  using NoiseModelPtr = gtsam::noiseModel::Base::shared_ptr;

public:
  lie_euler_integrator_factor_t(gtsam::Key xt0_key, gtsam::Key xt1_key, const NoiseModelPtr& cost_model,
                                const std::size_t dim, const double h = prx::simulation_step)
    : Base(cost_model, std::vector<gtsam::Key>(xt0_key, xt1_key)), _h(h), _dim(dim)
  {
    prx_assert(_h > 0, "[lie_euler_integrator_factor_t] h (sim step) must be greater than 0.");
  }

  virtual Eigen::VectorXd unwhitenedError(const gtsam::Values& values,
                                          boost::optional<std::vector<Eigen::MatrixXd>&> H = boost::none) const
  {
    const State x0{ values.at<State>(keys_[0]) };
    const State x1{ values.at<State>(keys_[1]) };
    Eigen::VectorXd error{ Eigen::VectorXd::Zero(_dim) };
    // if (this->active(values))
    // {
    //   const auto exp_A = State::Expmap(_h * x0);
    //   error = x1 - exp_A * x0;
    //   if (H)
    //   {
    //     (*H)[0] = -exp_A;
    //     (*H)[1] = Eigen::MatrixXd::Identity(_dim, _dim);
    //   }
    // }
    return error;
  }

private:
  const double _h;
  const std::size_t _dim;
};
}  // namespace fg
}  // namespace prx