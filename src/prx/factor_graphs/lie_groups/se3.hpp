#pragma once

#include <Eigen/Dense>
#include <Eigen/Core>
#include <gtsam/base/VectorSpace.h>
#include <gtsam/geometry/SO3.h>
#include <gtsam/geometry/Pose3.h>
#include "prx/factor_graphs/lie_groups/lie_operators.hpp"
// #include "prx/factor_graphs/lie_groups/screw_axis.hpp"

namespace prx
{
namespace fg
{

// aka a Pose.
// A pair translation/position, rotation where the rotation is a quaternion: (p,q).
// Mostly using the functionality of gtsam::Pose3 but we need access to the raw values of p and q for prx::space_t

class se3_t : public gtsam::LieGroup<se3_t, 6>
{
public:
  static constexpr Eigen::Index Dim = 6;
  static constexpr Eigen::Index dimension = 6;
  static constexpr Eigen::Index RowsAtCompileTime = 6;
  using Scalar = double;
  using Quaternion = Eigen::Quaterniond;
  using Position = Eigen::Vector<double, 3>;

  using gtsam::LieGroup<se3_t, 6>::inverse;  // version with derivative

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

  se3_t(const double qw, const double qx, const double qy, const double qz,  // no-lint
        const double x, const double y, const double z)
    : _quaternion(qw, qx, qy, qz), _position(x, y, z)
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

  Position operator*(const Position& x) const
  {
    return action(x);
  }

  Position action(const Position& pt,  // no-lint
                  gtsam::OptionalJacobian<3, 6> Hse3 = boost::none,
                  gtsam::OptionalJacobian<3, 3> Hpt = boost::none) const
  {
    if (Hse3 or Hpt)
    {
      const Eigen::Matrix3d R{ rotation_matrix() };
      if (Hse3)
      {
        Hse3->leftCols<3>() = R * gtsam::skewSymmetric(-pt[0], -pt[1], -pt[2]);
        Hse3->rightCols<3>() = R;
      }
      if (Hpt)
      {
        *Hpt = R;
      }
    }
    return _quaternion * pt + _position;
  }

  gtsam::Pose3 to_pose() const
  {
    const gtsam::Rot3 rot(_quaternion.toRotationMatrix());
    const gtsam::Pose3 pose(rot, _position);

    // PRX_DBG_VARS(_quaternion, _position.transpose());
    // PRX_DBG_VARS(pose);
    return pose;
  }

  template <typename ScrewAxis>
  static se3_t Expmap(const ScrewAxis& s, gtsam::OptionalJacobian<6, 6> H = boost::none)
  {
    const Eigen::Vector<double, 6> s_vec{ s };
    return se3_t(gtsam::Pose3::Expmap(s_vec, H));
  }

  static Eigen::Vector<double, 6> Logmap(const se3_t& s, gtsam::OptionalJacobian<6, 6> H = boost::none)
  {
    return gtsam::Pose3::Logmap(s.to_pose(), H);
  }

  Eigen::Matrix<double, 6, 6> AdjointMap() const
  {
    const Eigen::Matrix3d R{ _quaternion };
    const Eigen::Matrix3d A{ gtsam::skewSymmetric(_position) * R };
    Eigen::Matrix<double, 6, 6> adj;
    adj.block<3, 3>(0, 0) = R;
    adj.block<3, 3>(0, 3) = A;
    adj.block<3, 3>(3, 0) = Eigen::Matrix3d::Zero();
    adj.block<3, 3>(3, 3) = R;  //, Z_3x3, A, R;  // Gives [R 0; A R]
    return adj;
  }

  se3_t inverse() const
  {
    const Quaternion Rt{ _quaternion.inverse() };
    return se3_t(Rt, Rt * (-_position));
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

  struct ChartAtOrigin
  {
    static se3_t Retract(const Eigen::Vector<double, 6>& xi, ChartJacobian Hxi = boost::none)
    {
      return Expmap(xi, Hxi);
    }
    static Eigen::Vector<double, 6> Local(const se3_t& pose, ChartJacobian Hpose = boost::none)
    {
      return Logmap(pose, Hpose);
    }
  };

  inline Eigen::Transform<double, 3, Eigen::TransformTraits::Isometry> transform() const
  {
    Eigen::Transform<double, 3, Eigen::TransformTraits::Isometry> tf{};
    tf.linear() = rotation_matrix();
    tf.translation() = position();
    return tf;
  }

  inline Eigen::Matrix3d rotation_matrix() const
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
  // Convenient maps to the two components of the screw axis
  Quaternion _quaternion;
  Position _position;
};

}  // namespace fg
}  // namespace prx
namespace gtsam
{

template <>
struct traits<prx::fg::se3_t> : public gtsam::Testable<prx::fg::se3_t>,
                                // public gtsam::internal::VectorSpaceImpl<prx::fg::se3_t, 6>,
                                public internal::LieGroupTraits<prx::fg::se3_t>
{
  static constexpr Eigen::Index dimension = 6;
  static int GetDimension(const prx::fg::se3_t&)
  {
    return 6;
  }
};
}  // namespace gtsam