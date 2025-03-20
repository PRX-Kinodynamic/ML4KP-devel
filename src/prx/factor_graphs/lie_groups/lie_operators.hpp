#pragma once

#include <Eigen/Dense>
#include <Eigen/Core>

#include <unsupported/Eigen/MatrixFunctions>

namespace prx
{
namespace fg
{
namespace lie_operators
{

inline Eigen::Matrix3d hat(const Eigen::Vector3d& vec)
{
  Eigen::Matrix3d res{ Eigen::Matrix3d::Zero() };

  res(0, 1) = -vec[2];
  res(0, 2) = vec[1];

  res(1, 0) = vec[2];
  res(1, 2) = -vec[0];

  res(2, 0) = -vec[2];
  res(2, 1) = vec[0];
  return res;
}

// Compute \tau_x = Y (-) X, such that Y = X (+) \tau_x. \tau_x in local frame
template <typename LieGroup, Eigen::Index Dim = gtsam::traits<LieGroup>::dimension>
static inline Eigen::Vector<double, Dim> right_minus(const LieGroup& Y, const LieGroup& X,  // no-lint
                                                     gtsam::OptionalJacobian<Dim, Dim> Hy = boost::none,
                                                     gtsam::OptionalJacobian<Dim, Dim> Hx = boost::none)
{
  Eigen::Matrix<double, Dim, Dim> xI_H_x, tau_H_diff, diff_H_xI, diff_H_y;

  const LieGroup Xi{ X.inverse(Hx ? &xI_H_x : nullptr) };
  const LieGroup diff{ gtsam::traits<LieGroup>::Compose(Xi, Y, Hx ? &diff_H_xI : nullptr, Hy ? &diff_H_y : nullptr) };
  const Eigen::Vector<double, Dim> tau{ LieGroup::Logmap(diff, (Hx or Hy) ? &tau_H_diff : nullptr) };

  if (Hy)
  {
    *Hy = tau_H_diff * diff_H_y;
  }
  if (Hx)
  {
    *Hx = tau_H_diff * diff_H_xI * xI_H_x;
  }

  return tau;
}

template <typename LieGroup, Eigen::Index Dim = gtsam::traits<LieGroup>::dimension>
static inline double cost_to_go(const LieGroup x, const Eigen::Matrix3d& S, const LieGroup& goal)
{
  const Eigen::Vector<double, Dim> diff{ right_minus(x, goal) };
  return diff.transpose() * S * diff;
}

}  // namespace lie_operators
}  // namespace fg
}  // namespace prx