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
class smooth_factor_t : public gtsam::NoiseModelFactor2<Eigen::Vector<double, Dim>, Eigen::Vector<double, Dim>>
{
  using Base = gtsam::NoiseModelFactor2<Eigen::Vector<double, Dim>, Eigen::Vector<double, Dim>>;

public:
  using Vector = Eigen::Vector<double, Dim>;
  using Jacobian = Eigen::MatrixXd;
  using Error = gtsam::Vector;

  smooth_factor_t(const gtsam::Key& xi, const gtsam::Key& xj, const double lambda,
                  const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(cost_model, xi, xj), _lambda(lambda)
  {
  }

  Error evaluateError(const Vector& xi, const Vector& xj,  // no-lint
                      boost::optional<Jacobian&> H1 = boost::none,
                      boost::optional<Jacobian&> H2 = boost::none) const override
  {
    const Error error{ _lambda * (xi - xj) };

    if (H1)
    {
      *H1 = _lambda * Eigen::Matrix3d::Identity();
    }
    if (H2)
    {
      *H2 = -_lambda * Eigen::Matrix3d::Identity();
    }

    return error;
  }

  void eval_to_stream(gtsam::Values& values, std::ostream& os)
  {
    const Vector xi{ values.at<Vector>(this->template key<1>()) };
    const Vector xj{ values.at<Vector>(this->template key<2>()) };

    // const double
    os << xi.transpose() << " ";
    os << xj.transpose() << "\n";
  }

private:
  const double _lambda;
};

}  // namespace fg
}  // namespace prx