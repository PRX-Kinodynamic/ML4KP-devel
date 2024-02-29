#pragma once
#include <array>
#include <numeric>
#include <gtsam/config.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include "prx/utilities/defs.hpp"

namespace prx
{
namespace fg
{

template <Eigen::Index Dimension>
class euclidean_distance_factor_t
  : public gtsam::NoiseModelFactor2<Eigen::Vector<double, Dimension>, Eigen::Vector<double, Dimension>>
{
public:
  using Distance = Eigen::Vector<double, 1>;
  using Point = Eigen::Vector<double, Dimension>;
  using Error = Eigen::VectorXd;
  using OptJacobian = boost::optional<Eigen::MatrixXd&>;
  using Jacobian = Eigen::Matrix<double, 1, Dimension>;
  using Base = gtsam::NoiseModelFactor2<Point, Point>;

  euclidean_distance_factor_t(const gtsam::Key& p0, const gtsam::Key& p1, const double distance,
                              const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(cost_model, p0, p1), _distance(distance * distance)
  {
  }

  Error evaluateError(const Point& p0, const Point& p1, OptJacobian H0 = boost::none,
                      OptJacobian H1 = boost::none) const override
  {
    const Point diff{ p0 - p1 };
    const Distance distance(diff.squaredNorm() - _distance);

    if (H0)
    {
      *H0 = 2 * Point::Ones().transpose();
      // PRX_DEBUG_VAR_1(*H0);
    }
    if (H1)
    {
      *H1 = -2 * Point::Ones().transpose();
      // PRX_DEBUG_VAR_1(*H1);
    }
    // PRX_DEBUG_VAR_2(p0.transpose(), p1.transpose());
    // PRX_DEBUG_VAR_1(distance);
    return distance;
  }

  void eval_to_stream(gtsam::Values& values, std::ostream& os)
  {
    const Point xi{ values.at<Point>(this->template key<1>()) };
    const Point xj{ values.at<Point>(this->template key<2>()) };

    // const double
    os << std::sqrt(_distance) << " ";  // 1,
    os << xi.transpose() << " ";        // 2, Dimension
    os << xj.transpose() << " ";        //
  }

private:
  const double _distance;
};

}  // namespace fg
}  // namespace prx