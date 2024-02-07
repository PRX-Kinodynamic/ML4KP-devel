#pragma once

#include <Eigen/Dense>
#include <Eigen/Core>
#include "prx/factor_graphs/factors/lie_operators.hpp"

namespace prx
{
namespace fg
{
class SO3_t : public Eigen::Matrix3d
{
public:
  using RotationMat = Eigen::Matrix3d;
  static constexpr Eigen::Index Dim = 3;
  SO3_t(const RotationMat base) : RotationMat(base)
  {
  }

  SO3_t() : SO3_t(RotationMat::Zero())
  {
  }

  virtual ~SO3_t()
  {
  }

  template <typename Rotation>
  static void exp(Rotation& rot, const Eigen::Vector3d& x)
  {
    const double x_norm{ x.norm() };
    const Eigen::Matrix3d X{ lie_operators::hat(x) };
    rot = RotationMat::Identity() +          //
          (std::sin(x_norm) / x_norm) * X +  //
          ((1.0 - std::cos(x_norm)) / (x_norm * x_norm)) * X * X;
    PRX_DEBUG_VAR_1(x_norm);
    PRX_DEBUG_VAR_1(X);
    PRX_DEBUG_VAR_1(rot);
  }

  static RotationMat exp(const Eigen::Vector3d& x)
  {
    RotationMat rot{ RotationMat::Identity() };
    SO3_t::exp(rot, x);
    return rot;
  }

  // Jacobian left. J_l = J_r^T
  static RotationMat jacobian(const Eigen::Vector3d& x)
  {
    const double x_norm{ x.norm() };
    const Eigen::Matrix3d X{ lie_operators::hat(x) };
    return RotationMat::Identity() +                             //
           ((1.0 - std::cos(x_norm)) / (x_norm * x_norm)) * X +  //
           ((x_norm - std::sin(x_norm)) / std::pow(x_norm, 3)) * X * X;
  }

  friend std::ostream& operator<<(std::ostream& os, const SO3_t& obj)
  {
    const Eigen::Quaterniond q{ obj };
    os << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " ";
    return os;
  }
};
}  // namespace fg
}  // namespace prx