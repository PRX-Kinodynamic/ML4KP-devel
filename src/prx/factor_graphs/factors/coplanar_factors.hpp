#pragma once
#include <array>
#include <numeric>
#include <gtsam/config.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "prx/factor_graphs/factors/noise_model_factor.hpp"

namespace prx
{
namespace fg
{
class coplanar_4_point3D_factor_t : public noise_model_1p4_factor_t<1, 3, 3, 3, 3>
{
  using Base = noise_model_1p4_factor_t<1, 3, 3, 3, 3>;

public:
  using Coplanarity = Base::Xerr;  // fg::camera::Pixel;
  using Point3D = Base::X0;        // fg::camera::Pixel;

  coplanar_4_point3D_factor_t(gtsam::Key key_x0, gtsam::Key key_x1, gtsam::Key key_x2, gtsam::Key key_x3,
                              const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(key_x0, key_x1, key_x2, key_x3, cost_model)
  {
  }

  // coplanarity for 3d points is
  // [(x_{2}-x_{1})\times (x_{4}-x_{1})] \cdot (x_{3}-x_{1})=0
  virtual Coplanarity compute_error(const X0& x0, const X1& x1, const X2& x2, const X3& x3) const override
  {
    return ((x1 - x0).cross(x3 - x0)).transpose() * (x2 - x0);
  }
};

class angle_between_3_3d_points_factor_t : public noise_model_1p3_factor_t<1, 3, 3, 3>
{
  using Base = noise_model_1p3_factor_t<1, 3, 3, 3>;

public:
  using Error = Base::Xerr;  // fg::camera::Pixel;
  using Point3D = Base::X0;  // fg::camera::Pixel;

  angle_between_3_3d_points_factor_t(const double angle, gtsam::Key key_x0, gtsam::Key key_x1, gtsam::Key key_x2,
                                     const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(key_x0, key_x1, key_x2, cost_model), _angle(angle)
  {
  }

  // Given 3 points, compute 2 vectors: p0 = x1 - x0; p1 = x2 - x0;
  // p0 \cdot p1 = |p0| \cdot |p1| \cdot cos(angle)
  // Error = p0 \cdot p1 - |p0| \cdot |p1| \cdot cos(angle)
  virtual Error compute_error(const X0& x0, const X1& x1, const X2& x2) const override
  {
    const Eigen::Vector3d p0{ x1 - x0 };
    const Eigen::Vector3d p1{ x2 - x0 };

    return Error{ p0.dot(p1) - p0.norm() * p1.norm() * std::cos(_angle) };
  }

private:
  const double _angle;
};
}  // namespace fg
}  // namespace prx