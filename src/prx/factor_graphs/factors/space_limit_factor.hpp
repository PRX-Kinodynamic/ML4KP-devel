#pragma once

#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include <iostream>
#include <boost/optional.hpp>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "prx/utilities/defs.hpp"
#include "prx/utilities/spaces/space.hpp"

#include "prx/factor_graphs/utilities/prx_symbols.hpp"

namespace prx
{

/**
 * space_limit_factor_t is a class which enforces limits to states
 */
template <Eigen::Index DIM>
class space_limit_factor_t : public gtsam::NoiseModelFactor1<Eigen::Vector<double, DIM>>
{
  using state_t = Eigen::Vector<double, DIM>;
  using This = space_limit_factor_t;
  using Base = gtsam::NoiseModelFactor1<state_t>;
  using partial_fn = std::function<state_t(const state_t&)>;

public:
  /**
   * Construct from joint limits
   * @param q_key joint value key
   * @param cost_model noise model
   * @param lower_limit joint lower limit
   * @param upper_limit joint upper limit
   */
  space_limit_factor_t(gtsam::Key q_key, const gtsam::noiseModel::Base::shared_ptr& cost_model, const space_t* ss,
                       double _epsilon = 0.001)
    : Base(cost_model, q_key)
    , _upper_bound(Eigen::VectorXd::Zero(ss->get_dimension()))
    , _lower_bound(Eigen::VectorXd::Zero(ss->get_dimension()))
    , _derivative(0.01)
  {
    _ss = ss;
    epsilon = _epsilon;
    ss->copy(_upper_bound, _ss->get_upper_bounds());
    ss->copy(_lower_bound, _ss->get_lower_bounds());
    _derivative.model = [&](const state_t& xi) { return compute_error(xi); };
  }

  virtual ~space_limit_factor_t()
  {
  }

public:
  /**
   * Evaluate joint limit errors
   *
   * @param q joint value
   */
  Eigen::VectorXd evaluateError(const state_t& state, boost::optional<gtsam::Matrix&> H_q = boost::none) const override
  {
    // const std::size_t ss_dim{ _ss->get_dimension() };
    // Eigen::VectorXd error = Eigen::VectorXd::Zero(ss_dim);
    // Eigen::MatrixXd H_qp = Eigen::MatrixXd::Zero(ss_dim, ss_dim);

    const Eigen::VectorXd error{ compute_error(state) };
    if (H_q)
    {
      // _derivative.model = [&](const Eigen::VectorXd& xi) { return compute_error(xi); };
      *H_q = _derivative(state);
    }
    return error;
  }

  Eigen::VectorXd compute_error(const Eigen::VectorXd& state) const
  {
    // const std::size_t ss_dim{ _ss->get_dimension() };

    // The return of this is a weird type... Eigen::Array of X bools? auto takes care of that...
    auto less = _lower_bound.array() > state.array();
    auto greater = state.array() > _upper_bound.array();

    // The rest are const vects
    const state_t diff_lower{ state - _lower_bound };
    const state_t diff_upper{ state - _upper_bound };
    const state_t res_lower{ diff_lower.array() * less.template cast<double>() };
    const state_t res_upper{ diff_upper.array() * greater.template cast<double>() };
    const state_t res{ res_lower + res_upper };
    return res;
  }

  //// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override
  {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print contents
  void print(const std::string& s = "", const gtsam::KeyFormatter& kf = gtsam::DefaultKeyFormatter) const override
  {
    std::cout << s << "space_limit_factor";
    Base::print("", kf);
  }

private:
  const space_t* _ss;
  double epsilon;
  state_t _upper_bound;
  state_t _lower_bound;
  mutable math::first_order_derivative_t<partial_fn, state_t, 4> _derivative;
};

}  // namespace prx
