#pragma once

#include <Eigen/Dense>
#include <Eigen/Core>
// #include "prx/factor_graphs/factors/lie_operators.hpp"

inline std::ostream& operator<<(std::ostream& os, const gtsam::SO3& obj)
{
  const Eigen::Matrix3d m{ obj.matrix() };
  for (int r = 0; r < m.rows(); ++r)
  {
    for (int c = 0; c < m.cols(); ++c)
    {
      os << m(r, c) << " ";
    }
  }
  return os;
}
namespace prx
{
namespace fg
{
// class SO3_t : public gtsam::SO3
// {
//   using Base = gtsam::SO3;
//   using RotationType = Eigen::Matrix3d;

// public:
//   static constexpr Eigen::Index Dim = 3;
//   static constexpr Eigen::Index dimension = 3;

//   SO3_t(RotationType rotation) : Base(rotation)
//   {
//   }

//   SO3_t() : Base(RotationType::Identity())
//   {
//   }

//   virtual ~SO3_t()
//   {
//   }

//   // SO3_t operator*(const SO3_t& other) const
//   // {
//   //   const SO3_t result{ matrix_ * other.matrix_ };
//   //   return std::move(result);
//   // }

//   // template <typename RotationIn>
//   // static void Expmap(RotationIn& rot, const Eigen::Vector3d& x)
//   // {
//   //   const double x_norm{ x.norm() };
//   //   const Eigen::Matrix3d X{ lie_operators::hat(x) };
//   //   const Eigen::Matrix3d R{ Eigen::Matrix3d::Identity() +      //
//   //                            (std::sin(x_norm) / x_norm) * X +  //
//   //                            ((1.0 - std::cos(x_norm)) / (x_norm * x_norm)) * X * X };
//   //   const RotationIn res{ R };
//   //   rot = std::move(res);
//   // }

//   // static SO3_t Expmap(const Eigen::Vector3d& x)
//   // {
//   //   SO3_t rot;
//   //   SO3_t::Expmap(rot, x);
//   //   return rot;
//   // }

//   // static Eigen::Vector3d Logmap(const SO3_t& s, gtsam::OptionalJacobian<6, 6> H = boost::none)
//   // {
//   //   return ScrewAxis(gtsam::SO3::Logmap(s.to_pose(), H));
//   // }

//   // Jacobian left. J_l = J_r^T
//   // static RotationType jacobian(const Eigen::Vector3d& x)
//   // {
//   //   const double x_norm{ x.norm() };
//   //   const Eigen::Matrix3d X{ lie_operators::hat(x) };
//   //   const Eigen::Matrix3d res{ Eigen::Matrix3d::Identity() +                         //
//   //                              ((1.0 - std::cos(x_norm)) / (x_norm * x_norm)) * X +  //
//   //                              ((x_norm - std::sin(x_norm)) / std::pow(x_norm, 3)) * X * X };
//   //   return RotationType{ res };
//   // }

gtsam::SO3 random_SO3()
{
  using RotationType = Eigen::Matrix3d;
  const double x1{ prx::uniform_random(0.0, 1.0) };
  const double x2{ prx::uniform_random(0.0, 1.0) };
  const double x3{ prx::uniform_random(0.0, 1.0) };
  const double theta{ 2.0 * prx::constants::pi * x1 };
  const double phi{ 2.0 * prx::constants::pi * x2 };
  const double z{ x3 };

  const double v1{ std::cos(phi) * std::sqrt(z) };
  const double v2{ std::sin(phi) * std::sqrt(z) };
  const double v3{ std::sqrt(1.0 - z) };
  const Eigen::Vector3d V(v1, v2, v3);

  const Eigen::Rotation2D<double> R2(theta);
  RotationType R3{ RotationType::Identity() };
  R3.topLeftCorner<2, 2>() = R2.matrix();
  const RotationType M{ (2.0 * V * V.transpose() - RotationType::Identity()) * R3 };
  return std::move(gtsam::SO3(M));
}
//   // static void random(SO3_t& rotation)
//   // {
//   //   const double s{ prx::uniform_random(0.0, 1.0) };
//   //   const double sigma1{ std::sqrt(1.0 - s) };
//   //   const double sigma2{ std::sqrt(s) };
//   //   const double theta1{ 2.0 * prx::constants::pi * prx::uniform_random(0.0, 1.0) };
//   //   const double theta2{ 2.0 * prx::constants::pi * prx::uniform_random(0.0, 1.0) };
//   //   const double w{ std::cos(theta2) * sigma2 };
//   //   const double x{ std::sin(theta1) * sigma1 };
//   //   const double y{ std::cos(theta1) * sigma1 };
//   //   const double z{ std::sin(theta2) * sigma2 };
//   //   const Eigen::Quaterniond q{ w, x, y, z };

//   //   const SO3_t rot{ q.toRotationMatrix() };
//   //   rotation = std::move(rot);
//   // }

//   // static SO3_t random()
//   // {
//   //   SO3_t rotation;
//   //   random(rotation);
//   //   return std::move(rotation);
//   // }

// std::ostream& operator<<(std::ostream& out, const Fraction& f)

//   void print(const std::string& str = "") const
//   {
//     std::cout << (*this);
//   }

// private:
//   // RotationType _rotation;
// };
}  // namespace fg
}  // namespace prx

// namespace gtsam
// {
// template <>
// struct traits<prx::fg::SO3_t> : public internal::LieGroup<prx::fg::SO3_t>
// {
//   static constexpr Eigen::Index dimension = 3;
//   static constexpr int GetDimension(const prx::fg::SO3_t&)
//   {
//     return dimension;
//   }
// };
// }  // namespace gtsam