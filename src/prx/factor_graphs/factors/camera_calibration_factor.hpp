#pragma once
#include <array>
#include <numeric>
#include <gtsam/config.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "prx/external/aruco_nano.h"

#include "prx/simulation/plants/types/linear_time_variant.hpp"

#include "prx/utilities/math/first_order_derivative.hpp"
#include "prx/utilities/math/math_functions.hpp"

#include "prx/factor_graphs/factors/noise_model_factor.hpp"
#include "prx/factor_graphs/utilities/perception/camera.hpp"
#include "prx/factor_graphs/utilities/symbols_factory.hpp"

namespace prx
{
namespace fg
{

class camera_feature_factor_t : public noise_model_6factor_t<2, 9, 3, 3, 3, 1>
{
  using Base = noise_model_6factor_t<2, 9, 3, 3, 3, 1>;

public:
  using Pixel = Base::X0;                 // fg::camera::Pixel;
  using CameraWithDistortion = Base::X1;  // fg::camera::CameraWithDistortion;
  using Rotation = Base::X2;              // Eigen::Vector3d
  using Position = Base::X3;              // fg::camera::Position;
  using Translation = Base::X4;           // Eigen::Vector3d
  using S = Base::X5;                     // Eigen::Vector1d
  camera_feature_factor_t(gtsam::Key feature_key, gtsam::Key camera_key, gtsam::Key rot_key, gtsam::Key pos_key,
                          gtsam::Key tr_key, gtsam::Key s_key, const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(feature_key, camera_key, rot_key, pos_key, tr_key, s_key, cost_model)
  {
  }

  virtual Pixel compute_error(const Pixel& feature, const CameraWithDistortion& camera, const Rotation& r,
                              const Position& p, const Translation& t, const S& s) const override
  {
    const Eigen::Matrix3d R{ Eigen::AngleAxisd(r[0], Rotation::UnitX()) *  // no-lint
                             Eigen::AngleAxisd(r[1], Rotation::UnitY()) *  // no-lint
                             Eigen::AngleAxisd(r[2], Rotation::UnitZ()) };
    Position xyz = R * p + t;
    Pixel uv{ fg::camera::real_lense_model(xyz, camera) };
    return feature - uv * s[0];
  }
};

}  // namespace fg
}  // namespace prx