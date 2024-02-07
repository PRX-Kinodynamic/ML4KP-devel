#pragma once
#include <array>
#include <numeric>
#include <gtsam/config.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "prx/utilities/math/math_functions.hpp"
#include "prx/utilities/math/first_order_derivative.hpp"

#include "prx/factor_graphs/factors/noise_model_factor.hpp"

namespace prx
{

namespace fg
{
// Point_global = T * Point_local; T = [R|t] (+ 0's & 1's on bottom row)
// Given a known offset between two points and the position of each point in global frame,
// Error: Point_local - known_offset;
class point_offset_in_local_frame_factor_t : public noise_model_1p3_factor_t<3, 9, 3, 3>
{
  using Base = noise_model_1p3_factor_t<3, 9, 3, 3>;

public:
  using Error = typename Base::Xerr;
  using Rot_vec = typename Base::X0;
  using Translation = typename Base::X1;
  using Point = typename Base::X2;

  point_offset_in_local_frame_factor_t(const Point offset, gtsam::Key key_rot_vec, gtsam::Key key_translate,
                                       gtsam::Key key_point_global,
                                       const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(key_rot_vec, key_translate, key_point_global, cost_model), _offset(offset)
  {
  }

  virtual Error compute_error(const Rot_vec& rot_vec, const Translation& translation,
                              const Point& point_global) const override
  {
    const Eigen::Matrix3d R{ rot_vec.reshaped(3, 3) };
    Eigen::Transform<double, 3, Eigen::TransformTraits::Isometry> transform;
    transform.linear() = R;
    transform.translation() = translation;

    const Point point_local{ transform * point_global };
    return point_local - _offset;
  }

private:
  const Point _offset;
};

// Point_global = T * Point_local; T = [R|t] (+ 0's & 1's on bottom row); R from a quat
// Given a known offset between two points and the position of each point in global frame,
// Error: Point_local - known_offset;
class point_offset_in_local_frame_quat_factor_t : public noise_model_1p3_factor_t<3, 4, 3, 3>
{
  using Base = noise_model_1p3_factor_t<3, 4, 3, 3>;

public:
  using Error = typename Base::Xerr;
  using QuatVec = typename Base::X0;
  using Translation = typename Base::X1;
  using Point = typename Base::X2;

  point_offset_in_local_frame_quat_factor_t(const Point offset, gtsam::Key key_rot_vec, gtsam::Key key_translate,
                                            gtsam::Key key_point_global,
                                            const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(key_rot_vec, key_translate, key_point_global, cost_model), _offset(offset)
  {
  }

  virtual Error compute_error(const QuatVec& rot_vec, const Translation& translation,
                              const Point& point_global) const override
  {
    const Eigen::Matrix3d R{ rot_vec.reshaped(3, 3) };
    Eigen::Transform<double, 3, Eigen::TransformTraits::Isometry> transform;
    transform.linear() = R;
    transform.translation() = translation;

    const Point point_local{ transform * point_global };
    return point_local - _offset;
  }

private:
  const Point _offset;
};

// Given two points {p0,p1}, and a known offset such that p0=offset+p1, then:
// Error: p0 - offset + p1;
class point_offset_in_global_frame_factor_t : public noise_model_2factor_t<3, 3>
{
  using Base = noise_model_2factor_t<3, 3>;

public:
  using Point = typename Base::X0;

  point_offset_in_global_frame_factor_t(const Point offset, gtsam::Key key_p0, gtsam::Key key_p1,
                                        const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(key_p0, key_p1, cost_model), _offset(offset)
  {
  }

  // ToDo: jacobians are easy to get analitically
  virtual Point compute_error(const Point& p0, const Point& p1) const override
  {
    return p0 - _offset + p1;
  }

private:
  const Point _offset;
};
// *Force* a rotation vector (Mat as vector) to be Rotation Matrix
// R^t = R^-1
class rotation_vector_matrix_factor_t : public noise_model_1p1_factor_t<9, 9>
{
  using Base = noise_model_1p1_factor_t<9, 9>;

public:
  using Error = typename Base::Xerr;
  using Rot_vec = typename Base::X0;

  rotation_vector_matrix_factor_t(gtsam::Key key_rot_vec, const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(key_rot_vec, cost_model)
  {
  }

  virtual Error compute_error(const Rot_vec& rot_vec) const override
  {
    const Eigen::Matrix3d R{ rot_vec.reshaped(3, 3) };
    const Eigen::Matrix3d Rtra{ R.transpose() };
    const Eigen::Matrix3d Rinv{ R.inverse() };
    return Rtra.reshaped(9, 1) - Rinv.reshaped(9, 1);
  }

private:
};
}  // namespace fg
}  // namespace prx