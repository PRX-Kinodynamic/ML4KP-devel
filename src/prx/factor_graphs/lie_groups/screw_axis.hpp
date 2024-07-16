#pragma once

#include <Eigen/Dense>
#include <Eigen/Core>
#include <gtsam/base/VectorSpace.h>
#include <gtsam/geometry/SO3.h>
#include <gtsam/geometry/Pose3.h>
#include "prx/factor_graphs/lie_groups/lie_operators.hpp"
#include "prx/factor_graphs/lie_groups/se3.hpp"
namespace prx
{
namespace fg
{
// A Screw axis S = [\omega, v], where a Twist V = S \dot{\Theta}
class screw_axis_t : public gtsam::LieGroup<screw_axis_t, 6>
{
public:
  static constexpr Eigen::Index Dim = 6;
  static constexpr Eigen::Index dimension = 6;
  static constexpr Eigen::Index RowsAtCompileTime = 6;

  using OmegaV = Eigen::Vector<double, 6>;
  using Twist = Eigen::Vector<double, 6>;
  using Rotation = Eigen::Matrix3d;
  using SE3 = se3_t;
  using Omega = Eigen::Vector3d;
  using Velocity = Eigen::Vector3d;
  using Scalar = double;

  screw_axis_t() : screw_axis_t(Eigen::Vector<double, 6>::Zero())
  {
  }

  screw_axis_t(const screw_axis_t&) = default;

  // This constructor allows you to construct screw_axis_t from Eigen expressions
  template <typename OtherDerived>
  screw_axis_t(const Eigen::MatrixBase<OtherDerived>& other) : _omega_v(other)
  {
  }

  virtual ~screw_axis_t()
  {
  }

  double& operator[](const std::size_t& idx)
  {
    return _omega_v[idx];
  }

  double operator[](const std::size_t& idx) const
  {
    return _omega_v[idx];
  }

  OmegaV vector() const
  {
    return _omega_v;
  }

  OmegaV& vector()
  {
    return _omega_v;
  }

  operator Eigen::Vector<double, 6>() const
  {
    return _omega_v;
  }

  // This method allows you to assign Eigen expressions to screw_axis_t
  template <typename OtherDerived>
  screw_axis_t& operator=(const Eigen::MatrixBase<OtherDerived>& other)
  {
    _omega_v = other;
    return *this;
  }

  template <typename Arithmetic, std::enable_if_t<std::is_arithmetic_v<Arithmetic>, bool> = true>
  screw_axis_t operator/(const Arithmetic& arithmetic) const
  {
    return screw_axis_t{ _omega_v / arithmetic };
  }

  template <typename OtherDerived>
  screw_axis_t operator/(const Eigen::MatrixBase<OtherDerived>& other) const
  {
    return screw_axis_t{ _omega_v / other };
  }

  template <typename Arithmetic, std::enable_if_t<std::is_arithmetic_v<Arithmetic>, bool> = true>
  screw_axis_t operator*(const Arithmetic& other) const
  {
    return screw_axis_t{ _omega_v * other };
  }

  template <typename OtherDerived>
  screw_axis_t operator*(const Eigen::MatrixBase<OtherDerived>& other) const
  {
    return screw_axis_t{ _omega_v * other };
  }

  template <typename Omega, typename Velocity>
  static screw_axis_t from_twist(const Omega omega, const Velocity velocity)
  {
    return from_twist((Twist() << omega, velocity).finished());
  }

  static screw_axis_t from_twist(const Twist twist)
  {
    screw_axis_t screw(twist);
    screw = screw / screw.theta_dot();
    return screw;
  }

  // Get the twist representation of this screw_axis
  Twist twist() const
  {
    return Twist(_omega_v * theta_dot());
  }

  inline bool is_omega_zero() const
  {
    return omega().isApproxToConstant(0.0);
  }

  // \dot{\Theta} such that S \cdot \dot{\Theta} = V
  double theta_dot() const
  {
    return is_omega_zero() ? v().norm() : omega().norm();
  }

  static SE3 Expmap(const screw_axis_t& s, gtsam::OptionalJacobian<6, 6> H = boost::none)
  {
    if (H)
    {
      (*H).block<3, 3>(0, 0) = gtsam::SO3::ExpmapDerivative(s.omega());
    }
    const gtsam::Rot3 rotation{ gtsam::SO3::Expmap(s.omega()) };
    const Eigen::Vector3d translation{ gtsam::SO3::LogmapDerivative(s.omega()) * s.v() };
    // PRX_DEBUG_VAR_1(s.omega().dot(s.omega()));
    // PRX_DEBUG_VAR_1(s.omega());
    // PRX_DEBUG_VAR_1(rotation);
    // PRX_DEBUG_VAR_1(rotation.toQuaternion());
    const SE3 se3{ rotation.toQuaternion(), translation };
    return se3;
  }

  // struct ChartAtOrigin
  // {
  //   static SE3 Retract(const Eigen::Vector<double, 6>& xi, ChartJacobian Hxi = boost::none)
  //   {
  //     return Expmap(xi, Hxi);
  //   }
  //   static Eigen::Vector<double, 6> Local(const screw_axis_t& screw_axis, ChartJacobian H = boost::none)
  //   {
  //     return Logmap(screw_axis, H);
  //   }
  // };

  Omega omega() const
  {
    return _omega_v.head(3);
  }

  Velocity v() const
  {
    return _omega_v.tail(3);
  }

  // auto& omega()
  // {
  //   return _omega_v.head(3);
  // }

  // auto& v()
  // {
  //   return _omega_v.tail(3);
  // }

  friend std::ostream& operator<<(std::ostream& os, const screw_axis_t& screw)
  {
    os << screw._omega_v.transpose();
    return os;
  }

  void print(const std::string& str = "") const
  {
    std::cout << str << " " << _omega_v.transpose();
  }

  bool equals(const screw_axis_t& other, double tol = 1e-8) const
  {
    return omega().isApprox(other.omega(), tol) && v().isApprox(other.v(), tol);
  }

private:
  // Convinient maps to the two components of the screw axis
  OmegaV _omega_v;
  // Eigen::Map<Eigen::Vector3d> _omega;
  // Eigen::Map<Eigen::Vector3d> _v;
};

}  // namespace fg
}  // namespace prx
namespace gtsam
{

template <>
struct traits<prx::fg::screw_axis_t> : public gtsam::Testable<prx::fg::screw_axis_t>,
                                       public gtsam::internal::VectorSpaceImpl<prx::fg::screw_axis_t, 6>
{
  static constexpr Eigen::Index dimension = 6;
  static int GetDimension(const prx::fg::screw_axis_t&)
  {
    return 6;
  }
};

prx::fg::screw_axis_t operator+(const prx::fg::screw_axis_t& s_A, const prx::fg::screw_axis_t& s_B)
{
  return prx::fg::screw_axis_t(s_A.vector() + s_B.vector());
}

prx::fg::screw_axis_t operator-(const prx::fg::screw_axis_t& s_A, const prx::fg::screw_axis_t& s_B)
{
  return prx::fg::screw_axis_t(s_A.vector() - s_B.vector());
}

// prx::fg::screw_axis_t operator+(const prx::fg::screw_axis_t& x, const Eigen::Vector<double, 6>& tangent)
// {
//   const prx::fg::screw_axis_t other{ tangent };
//   return x + other;
// }
}  // namespace gtsam