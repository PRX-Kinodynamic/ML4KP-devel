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

}  // namespace lie_operators
}  // namespace fg
}  // namespace prx