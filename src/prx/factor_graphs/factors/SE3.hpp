#pragma once

#include <Eigen/Dense>
#include <Eigen/Core>

#include "prx/factor_graphs/factors/SO3.hpp"
namespace prx
{
namespace fg
{
class SE3_t : public Eigen::Transform<double, 3, Eigen::TransformTraits::Isometry>
{
public:
  using Base = Eigen::Transform<double, 3, Eigen::TransformTraits::Isometry>;
  static constexpr Eigen::Index Dim = 6;

  SE3_t(const Base base) : Base(base)
  {
  }

  SE3_t()
  {
  }

  virtual ~SE3_t()
  {
  }

  friend std::ostream& operator<<(std::ostream& os, const SE3_t& obj)
  {
    const Eigen::Quaterniond q{ obj.rotation() };
    const Eigen::Vector3d t{ obj.translation() };
    os << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " ";
    os << t.transpose();
    return os;
  }
};
}  // namespace fg
}  // namespace prx