#pragma once

#include <Eigen/Dense>
#include <Eigen/Core>
#include <gtsam/base/VectorSpace.h>
#include <gtsam/geometry/SO3.h>
#include <gtsam/geometry/Pose2.h>
#include "prx/factor_graphs/lie_groups/lie_operators.hpp"
#include "prx/factor_graphs/lie_groups/screw_axis.hpp"

namespace prx
{
namespace fg
{

// aka a Pose.
// A pair translation/position, rotation where the rotation is a quaternion: (p,q).
// Mostly using the functionallity of gtsam::Pose2 but we need access to the raw values of p and q for prx::space_t
class SO2_t : public gtsam::LieGroup<SO2_t, 1>
{
public:
  static constexpr Eigen::Index Dim = 1;
  static constexpr Eigen::Index dimension = 1;
  static constexpr Eigen::Index RowsAtCompileTime = 1;
  using Scalar = double;
  using Angle = double;

  using Rotation = Eigen::Rotation2D<double>;
  using RotationMatrix = Eigen::Matrix2d;

  using gtsam::LieGroup<SO2_t, 1>::inverse;  // version with derivative

  SO2_t() : SO2_t(0)
  {
  }

  SO2_t(const SO2_t&) = default;

  virtual ~SO2_t()
  {
  }

  SO2_t(const double theta) : _angle(bound(theta))
  {
  }

  SO2_t(const gtsam::Rot2 rot) : _angle(rot.theta())
  {
  }

  SO2_t(const Eigen::Matrix2d& rot) : _angle(std::atan2(rot(1, 0), rot(0, 0)))
  {
  }

  static double bound(const double theta)
  {
    return std::atan2(std::sin(theta), std::cos(theta));
  }

  static SO2_t Zero()  // Equivalent to Eigen's Zero
  {
    return SO2_t(0);
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
        return _angle;
      default:
        prx_throw("Index [" << idx << "] out of range");
    }
  }

  SO2_t operator*(const SO2_t& other) const
  {
    const Eigen::Matrix2d rot{ rotation<Eigen::Matrix2d>() };
    return SO2_t(rot * other.rotation<Eigen::Matrix2d>());
  }

  operator gtsam::Rot2() const
  {
    return gtsam::Rot2(_angle);
  }

  template <typename Vector>
  static SO2_t Expmap(const Vector& v, gtsam::OptionalJacobian<1, 1> H = boost::none)
  {
    return SO2_t(gtsam::Rot2::Expmap(v, H).theta());
  }

  Eigen::Matrix<double, 1, 1> AdjointMap() const
  {
    return Eigen::Matrix<double, 1, 1>::Identity();
  }

  // GTSAM_EXPORT static Vector3 Logmap(const Pose2& p, ChartJacobian H = boost::none);
  static Eigen::Vector<double, 1> Logmap(const SO2_t& x, gtsam::OptionalJacobian<1, 1> H = boost::none)
  {
    const gtsam::Rot2 xp{ static_cast<gtsam::Rot2>(x) };
    return gtsam::Rot2::Logmap(xp, H);
  }

  SO2_t inverse() const
  {
    const RotationMatrix Rt{ rotation<Eigen::Matrix2d>().transpose() };
    return SO2_t(rotation<Eigen::Matrix2d>().transpose());
  }

  friend std::ostream& operator<<(std::ostream& os, const SO2_t& s)
  {
    os << s.angle() << " ";
    return os;
  }

  struct ChartAtOrigin
  {
    static SO2_t Retract(const Eigen::Vector<double, 1>& xi, ChartJacobian Hxi = boost::none)
    {
      return Expmap(xi, Hxi);
    }
    static Eigen::Vector<double, 1> Local(const SO2_t& pose, ChartJacobian Hpose = boost::none)
    {
      return Logmap(pose, Hpose);
    }
  };
  void print(const std::string& str = "") const
  {
    std::cout << str << " " << (*this);
  }

  bool equals(const SO2_t& other, double tol = 1e-8) const
  {
    return (other.angle() - _angle) < tol;
  }

  inline Angle angle() const
  {
    return _angle;
  }

  inline Angle& angle()
  {
    return _angle;
  }

  template <typename RotationOut>
  inline RotationOut rotation() const
  {
    const Eigen::Vector<double, 1> vec{ angle() };
    const Eigen::Matrix3d mat3d{ euler_to_rotation<Eigen::Matrix3d>(vec, "Z") };
    return RotationOut{ mat3d.block<2, 2>(0, 0) };
  }

  static inline SO2_t random()
  {
    const double pim{ -prx::constants::pi };
    const double& pi{ prx::constants::pi };
    const double theta{ prx::uniform_random<double>(pim, pi) };

    return std::move(SO2_t(theta));
  }

private:
  // Convenient maps to the two components of the screw axis
  Angle _angle;
};

}  // namespace fg
}  // namespace prx
namespace gtsam
{

template <>
struct traits<prx::fg::SO2_t> : public gtsam::Testable<prx::fg::SO2_t>, public internal::LieGroupTraits<prx::fg::SO2_t>
{
  static constexpr Eigen::Index dimension = 1;
  static int GetDimension(const prx::fg::SO2_t&)
  {
    return 1;
  }
};

prx::fg::SO2_t operator+(const prx::fg::SO2_t& x, const Eigen::Vector<double, 1>& tangent)
{
  const prx::fg::SO2_t exmap{ prx::fg::SO2_t::Expmap(tangent) };
  return gtsam::traits<prx::fg::SO2_t>::Compose(x, exmap);
}

}  // namespace gtsam