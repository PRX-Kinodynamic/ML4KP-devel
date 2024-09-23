#pragma once

#include <Eigen/Dense>
#include <Eigen/Core>
#include <gtsam/base/VectorSpace.h>
#include <gtsam/geometry/SO3.h>
#include <gtsam/geometry/Pose2.h>
// #include "prx/factor_graphs/lie_groups/screw_axis.hpp"

namespace prx
{
namespace fg
{

// aka a Pose.
// A pair translation/position, rotation where the rotation is a quaternion: (p,q).
// Mostly using the functionallity of gtsam::Pose2 but we need access to the raw values of p and q for prx::space_t
class SE2_t : public gtsam::LieGroup<SE2_t, 3>
{
public:
  static constexpr Eigen::Index Dim = 3;
  static constexpr Eigen::Index dimension = 3;
  static constexpr Eigen::Index RowsAtCompileTime = 3;
  using Scalar = double;
  using Angle = double;
  using Translation = Eigen::Vector<double, 2>;

  using Rotation = Eigen::Rotation2D<double>;
  using RotationMatrix = Eigen::Matrix2d;

  using gtsam::LieGroup<SE2_t, 3>::inverse;  // version with derivative

  SE2_t() : SE2_t(0, 0, 0)
  {
  }

  SE2_t(const SE2_t&) = default;

  virtual ~SE2_t()
  {
  }

  SE2_t(const Translation translation, const double theta) : _translation(translation), _angle(theta)
  {
  }

  SE2_t(const Translation translation, const Rotation rotation) : SE2_t(translation, rotation.angle())
  {
  }

  SE2_t(const Translation translation, const RotationMatrix rotation) : SE2_t(translation, Rotation(rotation))
  {
  }

  SE2_t(const double x, const double y, const double theta) : SE2_t(Translation(x, y), theta)
  {
  }

  SE2_t(const gtsam::Pose2& pose) : SE2_t(pose.t(), pose.r().matrix())
  {
  }

  static SE2_t Zero()  // Equivalent to Eigen's Zero
  {
    return SE2_t(0, 0, 0);
  }

  static std::size_t size()
  {
    return Dim;
  }

  double& operator[](const std::size_t& idx)
  {
    switch (idx)
    {
      case 0:
        return _translation[0];
      case 1:
        return _translation[1];
      case 2:
        return _angle;
      default:
        prx_throw("Index [" << idx << "] out of range");
    }
  }

  double operator[](const std::size_t& idx) const
  {
    switch (idx)
    {
      case 0:
        return _translation[0];
      case 1:
        return _translation[1];
      case 2:
        return _angle;
      default:
        prx_throw("Index [" << idx << "] out of range");
    }
  }

  SE2_t operator*(const SE2_t& other) const
  {
    const Rotation rot{ rotation() };
    return SE2_t(translation() + rot * other.translation(), rot * other.rotation());
  }

  operator gtsam::Pose2() const
  {
    return gtsam::Pose2(gtsam::Rot2(_angle), translation());
  }

  template <typename ScrewAxis>
  static SE2_t Expmap(const ScrewAxis& s, gtsam::OptionalJacobian<3, 3> H = boost::none)
  {
    const Eigen::Vector<double, 3> s_vec{ s };
    return SE2_t(gtsam::Pose2::Expmap(s_vec, H));
  }

  // GTSAM_EXPORT static Vector3 Logmap(const Pose2& p, ChartJacobian H = boost::none);
  static Eigen::Vector<double, 3> Logmap(const SE2_t& s, gtsam::OptionalJacobian<3, 3> H = boost::none)
  {
    const gtsam::Pose2 pose2{ static_cast<gtsam::Pose2>(s) };
    return gtsam::Pose2::Logmap(pose2, H);
  }

  Eigen::Matrix<double, 3, 3> AdjointMap() const
  {
    Eigen::Matrix<double, 3, 3> adjM{ Eigen::Matrix<double, 3, 3>::Identity() };
    adjM.block<2, 2>(0, 0) = rotation();
    adjM(0, 2) = y();
    adjM(1, 2) = -x();
    return adjM;
  }

  SE2_t inverse() const
  {
    const RotationMatrix Rt{ rotation().transpose() };
    return SE2_t(-Rt * translation(), Rt);
  }

  friend std::ostream& operator<<(std::ostream& os, const SE2_t& s)
  {
    os << s.x() << " ";
    os << s.y() << " ";
    os << s.angle() << " ";
    return os;
  }

  struct ChartAtOrigin
  {
    static SE2_t Retract(const Eigen::Vector<double, 3>& xi, ChartJacobian Hxi = boost::none)
    {
      return Expmap(xi, Hxi);
    }
    static Eigen::Vector<double, 3> Local(const SE2_t& pose, ChartJacobian Hpose = boost::none)
    {
      return Logmap(pose, Hpose);
    }
  };
  void print(const std::string& str = "") const
  {
    std::cout << str << " " << (*this);
  }

  bool equals(const SE2_t& other, double tol = 1e-8) const
  {
    const RotationMatrix id_test{ rotation().transpose() * other.rotation() };
    return _translation.isApprox(other.translation(), tol) && id_test.isIdentity(1e-3);
  }

  inline Angle angle() const
  {
    return _angle;
  }

  inline Angle& angle()
  {
    return _angle;
  }

  inline RotationMatrix rotation() const
  {
    return Rotation(_angle).toRotationMatrix();
  }

  inline Translation translation() const
  {
    return _translation;
  }

  inline Translation& translation()
  {
    return _translation;
  }

  inline double x() const
  {
    return _translation[0];
  }

  inline double& x()
  {
    return _translation[0];
  }

  inline double y() const
  {
    return _translation[1];
  }

  inline double& y()
  {
    return _translation[1];
  }

private:
  // Convenient maps to the two components of the screw axis
  Angle _angle;
  Translation _translation;
};

}  // namespace fg
}  // namespace prx
namespace gtsam
{

template <>
struct traits<prx::fg::SE2_t> : public gtsam::Testable<prx::fg::SE2_t>,
                                // public gtsam::internal::VectorSpaceImpl<prx::fg::SE2_t, 6>,
                                public internal::LieGroupTraits<prx::fg::SE2_t>
{
  static constexpr Eigen::Index dimension = 3;
  static int GetDimension(const prx::fg::SE2_t&)
  {
    return 3;
  }
};

prx::fg::SE2_t operator+(const prx::fg::SE2_t& x, const Eigen::Vector3d& tangent)
{
  const prx::fg::SE2_t exmap{ prx::fg::SE2_t::Expmap(tangent) };
  return gtsam::traits<prx::fg::SE2_t>::Compose(x, exmap);
}

}  // namespace gtsam