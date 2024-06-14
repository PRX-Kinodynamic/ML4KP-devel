#pragma once
#include <array>
#include <numeric>
#include <gtsam/config.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace prx
{
namespace fg
{
template <typename State>
class quadratic_cost_factor_t : public gtsam::NoiseModelFactor1<State>
{
  using Base = gtsam::NoiseModelFactor1<State>;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

  static constexpr Eigen::Index Dim{ gtsam::traits<State>::dimension };
  using Matrix = Eigen::Matrix<double, Dim, Dim>;
  // using Vector = Eigen::Vector<double, Dim>;

public:
  quadratic_cost_factor_t(const gtsam::Key& key, const Matrix cost, const State& offset, const NoiseModel& cost_model)
    : Base(cost_model, key), _cost(cost), _offset(offset)
  {
  }
  quadratic_cost_factor_t(const gtsam::Key& key, const Matrix cost, const NoiseModel& cost_model)
    : quadratic_cost_factor_t(key, cost, State::Zero(), cost_model)
  {
  }

  virtual Eigen::VectorXd evaluateError(const State& x0,
                                        boost::optional<Eigen::MatrixXd&> H0 = boost::none) const override
  {
    const State xp{ x0 - _offset };

    const Eigen::VectorXd error{ xp.transpose() * _cost * xp };
    if (H0)
    {
      //.  2x2 * 2x1
      *H0 = (_cost * xp + _cost.transpose() * xp).transpose();
    }
    return error;
  }

private:
  // _Matrix _A;
  Matrix _cost;
  State _offset;
};
}  // namespace fg
}  // namespace prx