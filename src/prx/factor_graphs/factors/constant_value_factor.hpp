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

template <Eigen::Index Dim>
class constant_value_factor_t : public gtsam::NoiseModelFactor1<Eigen::Vector<double, Dim>>
{
  using Base = gtsam::NoiseModelFactor1<Eigen::Vector<double, Dim>>;

public:
  using Vector = Eigen::Vector<double, Dim>;
  using Jacobian = Eigen::MatrixXd;
  using Error = gtsam::Vector;

  constant_value_factor_t(const gtsam::Key& xi, const Vector value,
                          const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(cost_model, xi), _value(lambda)
  {
  }

  Error evaluateError(const Vector& xi,  // no-lint
                      boost::optional<Jacobian&> H1 = boost::none) const override
  {
    const Error error{ xi - _value };

    if (H1)
    {
      *H1 = Eigen::Matrix3d::Identity();
    }

    return error;
  }

  void eval_to_stream(gtsam::Values& values, std::ostream& os)
  {
    const Vector xi{ values.at<Vector>(this->template key<1>()) };

    // const double
    os << _value.transpose() << " " << xi.transpose() << " ";
  }

private:
  const Value _value;
};

}  // namespace fg
}  // namespace prx