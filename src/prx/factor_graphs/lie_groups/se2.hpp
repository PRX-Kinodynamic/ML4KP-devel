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
class SE2_t : public gtsam::LieGroup<SE2_t, 3>
{
  static const inline Eigen::Vector3d max{ 100, 100, prx::constants::pi };

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

  SE2_t(const Eigen::Vector3d vector) : _translation(vector.head(2)), _angle(vector[2])
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
    const Eigen::Matrix2d rot{ rotation<Eigen::Matrix2d>() };
    return SE2_t(translation() + rot * other.translation(), rot * other.rotation<Eigen::Matrix2d>());
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

  // This is the operator [Adj_V] for a vector V
  static Eigen::Matrix<double, 3, 3> adjoint_map(const Eigen::Vector3d& v)
  {
    const double& x{ v[0] };
    const double& y{ v[1] };
    const double& th{ v[2] };
    Eigen::Matrix<double, 3, 3> adjM;
    adjM << 0, -th, y, th, 0, -x, 0, 0, 0;
    return std::move(adjM);
  }

  // This is the adjoint for the current Pose/SE2
  static Eigen::Matrix<double, 3, 3> AdjointMap(const SE2_t& pose)
  {
    Eigen::Matrix<double, 3, 3> adjM{ Eigen::Matrix<double, 3, 3>::Identity() };
    adjM.block<2, 2>(0, 0) = pose.rotation<Eigen::Matrix2d>();
    adjM(0, 2) = pose.y();
    adjM(1, 2) = -pose.x();
    return adjM;
  }

  Eigen::Matrix<double, 3, 3> AdjointMap() const
  {
    return AdjointMap(*this);
  }

  Eigen::Vector3d adjoint(const Eigen::Vector3d& v,  // no-lint
                          gtsam::OptionalJacobian<3, 3> Hse2 = boost::none,
                          gtsam::OptionalJacobian<3, 3> Hv = boost::none) const
  {
    return adjoint(*this, v, Hse2, Hv);
  }

  template <typename Pose>
  static Eigen::Vector3d adjoint(const Pose& pose, const Eigen::Vector3d& v,  // no-lint
                                 gtsam::OptionalJacobian<3, 3> Hse2 = boost::none,
                                 gtsam::OptionalJacobian<3, 3> Hv = boost::none)
  {
    const Eigen::Matrix3d ad{ AdjointMap(pose) };
    if (Hse2)
      *Hse2 = -ad * adjoint_map(v);
    if (Hv)
      *Hv = ad;
    return ad * v;
  }

  SE2_t inverse() const
  {
    const RotationMatrix Rt{ rotation<Eigen::Matrix2d>().transpose() };
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
    const RotationMatrix id_test{ rotation<Eigen::Matrix2d>().transpose() * other.rotation<Eigen::Matrix2d>() };
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

  template <typename RotationOut>
  inline RotationOut rotation() const
  {
    const Eigen::Vector<double, 1> vec{ angle() };
    const Eigen::Matrix3d mat3d{ euler_to_rotation<Eigen::Matrix3d>(vec, "Z") };
    return RotationOut{ mat3d.block<2, 2>(0, 0) };
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

  inline Eigen::Vector3d vector() const
  {
    return std::move(Eigen::Vector3d(x(), y(), angle()));
  }

  static inline SE2_t random(const Eigen::Vector3d& min_bound = -max, const Eigen::Vector3d max_bound = max)
  {
    prx_assert(min_bound.size() == 3, "Min bounds size mismatch: expected 3 but got " << min_bound.size());
    prx_assert(max_bound.size() == 3, "Max bounds size mismatch: expected 3 but got " << max_bound.size());

    const double x{ prx::uniform_random<double>(min_bound[0], max_bound[0]) };
    const double y{ prx::uniform_random<double>(min_bound[1], max_bound[1]) };
    const double theta{ prx::uniform_random<double>(min_bound[2], max_bound[2]) };

    return std::move(SE2_t(x, y, theta));
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
struct traits<prx::fg::SE2_t> : public gtsam::Testable<prx::fg::SE2_t>, public internal::LieGroupTraits<prx::fg::SE2_t>
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