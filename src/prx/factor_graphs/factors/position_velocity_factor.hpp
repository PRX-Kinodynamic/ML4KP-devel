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

class position_velocity_factor_t
  : public gtsam::NoiseModelFactor3<Eigen::Vector<double, 3>, Eigen::Vector<double, 3>, Eigen::Vector<double, 3>>
{
  using Base = gtsam::NoiseModelFactor3<Eigen::Vector<double, 3>, Eigen::Vector<double, 3>, Eigen::Vector<double, 3>>;

public:
  using Position = Eigen::Vector3d;
  using Velocity = Eigen::Vector3d;
  using Jacobian = Eigen::MatrixXd;
  using Error = gtsam::Vector;

  position_velocity_factor_t(const gtsam::Key& xi, const gtsam::Key& xj, const gtsam::Key& vi, const double ti,
                             const double tj, const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(cost_model, xi, xj, vi), _dt(tj - ti), _ti(ti), _tj(tj)
  {
  }

  Error evaluateError(const Position& xi, const Position& xj, const Velocity& vj,
                      boost::optional<Jacobian&> H1 = boost::none, boost::optional<Jacobian&> H2 = boost::none,
                      boost::optional<Jacobian&> H3 = boost::none) const override
  {
    const Position xjp{ xi + vj * _dt };

    if (H1)
    {
      *H1 = Eigen::Matrix3d::Identity();
    }
    if (H2)
    {
      *H2 = -Eigen::Matrix3d::Identity();
    }
    if (H3)
    {
      *H3 = _dt * Eigen::Matrix3d::Identity();
    }

    return xjp - xj;
  }

  void eval_to_stream(gtsam::Values& values, std::ostream& os)
  {
    const Position xi{ values.at<Position>(key<1>()) };
    const Position xj{ values.at<Position>(key<2>()) };
    const Velocity vi{ values.at<Velocity>(key<3>()) };

    // const double
    os << _ti << " " << _tj << " ";  // 1,  2
    os << xi.transpose() << " ";     // 3,  4,  5
    os << xj.transpose() << " ";     // 6,  7,  8
    os << vi.transpose() << "\n";    // 9, 10, 11
  }

private:
  const double _dt, _ti, _tj;
};

}  // namespace fg
}  // namespace prx