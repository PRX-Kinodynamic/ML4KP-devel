#pragma once

#include <Eigen/Dense>
#include <Eigen/Core>
#include <gtsam/base/Lie.h>

namespace prx
{
template <typename LieType, int DimX = gtsam::traits<LieType>::dimension>
Eigen::Vector<double, DimX> TangentBetween(const LieType& x, const LieType& y,               // no-lint
                                           Eigen::Matrix<double, DimX, DimX>* Hx = nullptr,  // no-lint
                                           Eigen::Matrix<double, DimX, DimX>* Hy = nullptr)
{
  Eigen::Matrix<double, DimX, DimX> btw_H_x, btw_H_y;
  Eigen::Matrix<double, DimX, DimX> tg_H_btw;
  const LieType btw{ gtsam::traits<LieType>::Between(x, y, btw_H_x, btw_H_y) };
  const Eigen::Vector<double, DimX> tgbtw{ gtsam::traits<LieType>::Logmap(btw, tg_H_btw) };

  if (Hx)
  {
    *Hx = tg_H_btw * btw_H_x;
  }
  if (Hy)
  {
    *Hy = tg_H_btw * btw_H_y;
  }
  return tgbtw;
}

}  // namespace prx