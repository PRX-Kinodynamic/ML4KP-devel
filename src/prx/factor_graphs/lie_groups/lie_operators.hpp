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

// Equation 58 of https://arxiv.org/pdf/1812.01537.pdf
// Y = f(x) ;             Dy/Dx
// Z = g(Y) = g(f(x)) ;   Dz/Dx
// Dz/Dx = Dz/Dy * Dy/Dx
template <typename DZDX, typename DZDY, typename DYDX>
inline void chain_rule(DZDX& dzdx, const DZDY& dzdy, const DYDX& dydx)
{
  dzdx = dzdy * dydx;
}

// template <typename UV, typename DerivU, typename DerivV, typename DUDX, typename DVDX>
// inline void leibniz_rule(UV& uv, const U& u, const V& v, const DUDX& dudx, const DVDX& dvdx)
// {
//   dudx = deriv_u(x);
//   dvdx = deriv_v(x);
//   uv = leibniz_rule(uv, u, v, dudx, dvdx);
// }

}  // namespace lie_operators
}  // namespace fg
}  // namespace prx