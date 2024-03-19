#pragma once

#include <Eigen/Dense>
#include <Eigen/Core>
#include <gtsam/base/VectorSpace.h>
#include <gtsam/geometry/SO3.h>
#include <gtsam/geometry/Pose3.h>
#include "prx/factor_graphs/lie_groups/lie_operators.hpp"
#include "prx/factor_graphs/lie_groups/screw_axis.hpp"

namespace prx
{
namespace fg
{

// aka a Pose.
// A pair translation/position, rotation where the rotation is a quaternion: (p,q).
// Mostly using the functionallity of gtsam::Pose3 but we need access to the raw values of p and q for prx::space_t
class se3_t
{
public:
  static constexpr Eigen::Index Dim = 6;
  static constexpr Eigen::Index dimension = 6;

  using Quaternion = Eigen::Quaterniond;
  using Position = Eigen::Vector<double, 3>;

  se3_t() : se3_t(Quaternion::Identity(), Position::Zero())
  {
  }

  se3_t(const se3_t&) = default;

  virtual ~se3_t()
  {
  }

  // This constructor allows you to construct screw_axis_t from Eigen expressions
  template <typename Rotation, typename PositionDerived>
  se3_t(const Rotation& q_other, const Eigen::MatrixBase<PositionDerived>& p_other)
    : _quaternion(q_other), _position(p_other)
  {
  }

  se3_t(const gtsam::Pose3& pose) : se3_t(pose.rotation().toQuaternion(), pose.translation())
  {
  }

  double& operator[](const std::size_t& idx)
  {
    switch (idx)
    {
      case 0:
        return _quaternion.w();
      case 1:
        return _quaternion.x();
      case 2:
        return _quaternion.y();
      case 3:
        return _quaternion.z();
      case 4:
        return _position[0];
      case 5:
        return _position[1];
      case 6:
        return _position[2];
      default:
        prx_throw("Index [" << idx << "] out of range");
    }
  }

  double operator[](const std::size_t& idx) const
  {
    switch (idx)
    {
      case 0:
        return _quaternion.w();
      case 1:
        return _quaternion.x();
      case 2:
        return _quaternion.y();
      case 3:
        return _quaternion.z();
      case 4:
        return _position[0];
      case 5:
        return _position[1];
      case 6:
        return _position[2];
      default:
        prx_throw("Index [" << idx << "] out of range");
    }
  }

  se3_t operator*(const se3_t& other) const
  {
    const Position pos{ _quaternion * other.position() + _position };
    const se3_t result{ _quaternion * other.quaternion(), pos };
    return result;
  }

  gtsam::Pose3 to_pose() const
  {
    return gtsam::Pose3(gtsam::Rot3(_quaternion), _position);
  }

  template <typename ScrewAxis>
  static se3_t expmap(const ScrewAxis& s, gtsam::OptionalJacobian<6, 6> H = boost::none)
  {
    const Eigen::Vector<double, 6> s_vec{ s };
    return se3_t(gtsam::Pose3::Expmap(s_vec, H));
  }

  template <typename ScrewAxis>
  static ScrewAxis logmap(const se3_t& s, gtsam::OptionalJacobian<6, 6> H = boost::none)
  {
    return ScrewAxis(gtsam::Pose3::Logmap(s.to_pose(), H));
  }

  friend std::ostream& operator<<(std::ostream& os, const se3_t& screw)
  {
    const Quaternion& q{ screw.quaternion() };
    os << q.w() << " ";
    os << q.x() << " ";
    os << q.y() << " ";
    os << q.z() << " ";
    os << screw.position().transpose() << " ";
    return os;
  }

  void print(const std::string& str = "") const
  {
    std::cout << str << " " << (*this);
  }

  bool equals(const se3_t& other, double tol = 1e-8) const
  {
    return _position.isApprox(other.position(), tol) && _quaternion.isApprox(other.quaternion(), tol);
  }

  inline transform_t transform() const
  {
    return transform_t(_quaternion);
  }

  inline Eigen::Matrix3d matrix() const
  {
    return _quaternion.toRotationMatrix();
  }

  inline Quaternion quaternion() const
  {
    return _quaternion;
  }

  inline Quaternion& quaternion()
  {
    return _quaternion;
  }

  inline Position position() const
  {
    return _position;
  }

  inline Position& position()
  {
    return _position;
  }

private:
  // Convinient maps to the two components of the screw axis
  Quaternion _quaternion;
  Position _position;
};

}  // namespace fg
}  // namespace prx
namespace gtsam
{

template <>
struct traits<prx::fg::se3_t> : public gtsam::Testable<prx::fg::se3_t>,
                                public gtsam::internal::VectorSpaceImpl<prx::fg::se3_t, 6>
{
  static int GetDimension(const prx::fg::se3_t&)
  {
    return 6;
  }
};
}  // namespace gtsam