#pragma once
#include <array>
#include <numeric>
#include <gtsam/config.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "prx/utilities/math/math_functions.hpp"
#include "prx/factor_graphs/utilities/symbols_factory.hpp"
#include "prx/simulation/plants/types/linear_time_variant.hpp"

namespace prx
{

class bang_bang_factor_t : public gtsam::NoiseModelFactor1<Eigen::VectorXd>
{
private:
  using This = bang_bang_factor_t;
  using Base = gtsam::NoiseModelFactor1<Eigen::VectorXd>;
  space_t* ss;
  system_ptr_t sys_ptr;

public:
  bang_bang_factor_t(const gtsam::noiseModel::Base::shared_ptr& cost_model, gtsam::Key xt_key, system_ptr_t _sys_ptr,
                     double _ti)
    : Base(cost_model, xt_key)
  {
    sys_ptr = _sys_ptr;
  }

  virtual ~bang_bang_factor_t()
  {
  }

  Eigen::VectorXd compute_error(const Eigen::VectorXd& q) const
  {
    auto ss = sys_ptr->get_control_space();
    auto ss_dim = ss->get_dimension();

    Eigen::VectorXd error(ss_dim);
    for (int i = 0; i < ss_dim; ++i)
    {
      auto q = ss->at(i);
      if (q == 0.0 || q == ss->get_lower_bound(i) || q == ss->get_upper_bound(i))
      {
        error[i] = 0;
      }
      else if (q < 0.0)
      {
        error[i] = -1 * std::min(std::fabs(ss->get_lower_bound(i) - q), q);
      }
      else
      {
        error[i] = 1 * std::min(std::fabs(ss->get_upper_bound(i) - q), q);
      }
    }
    return error;
  }

  gtsam::Vector evaluateError(const Eigen::VectorXd& q,
                              boost::optional<gtsam::Matrix&> H_q = boost::none) const override
  {
    Eigen::VectorXd error = compute_error(q);

    if (H_q)
    {
      std::function<Eigen::VectorXd(Eigen::VectorXd)> fp =
          std::bind(&bang_bang_factor_t::compute_error, this, std::placeholders::_1);
      *H_q = math_functions::differentiate(fp, q);
    }
    return error;
  }
};
}  // namespace prx