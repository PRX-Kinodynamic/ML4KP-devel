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

class camera_to_world_factor_t : public noise_model_5factor_t<3, 3, 4, 5, 3>
{
  using Base = noise_model_5factor_t<3, 3, 4, 5, 3>;

public:
  using Pixel = Base::X0;
  using Translation = Base::X1;
  using Quaternion = Base::X2;
  using CameraParameters = Base::X3;
  using WorldPosition = Base::X4;

  camera_to_world_factor_t(gtsam::Key key_pixels, gtsam::Key key_translation, gtsam::Key key_orientation,
                           gtsam::Key key_params, gtsam::Key key_world,
                           const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(key_pixels, key_translation, key_orientation, key_params, key_world, cost_model)
  {
  }

  virtual Pixel compute_error(const Pixel& pixels, const Translation& translation, const Quaternion& quaternion,
                              const CameraParameters& cam_params, const WorldPosition& world) const override
  {
    Eigen::Matrix3d cam_mat{ Eigen::Matrix3d::Identity() };
    const Eigen::Quaternion quat{ quaternion[0], quaternion[1], quaternion[2], quaternion[3] };
    const Eigen::Matrix3d R{ quat };
    cam_mat(0, 0) = cam_params[0];
    cam_mat(1, 1) = cam_params[1];
    cam_mat(0, 2) = cam_params[2];
    cam_mat(1, 2) = cam_params[3];
    cam_mat(0, 1) = cam_params[4];

    // cam* R* pos + cam* T;
    Eigen::Vector3d computed_pixels{ cam_mat * R * world + cam_mat * translation };
    // PRX_DEBUG_VAR_1(pixels.transpose());
    // PRX_DEBUG_VAR_1(translation.transpose());
    // PRX_DEBUG_VAR_1(quat);
    // PRX_DEBUG_VAR_1(cam_mat);
    // PRX_DEBUG_VAR_1(world.transpose());
    // PRX_DEBUG_VAR_1(R);
    // PRX_DEBUG_VAR_1(computed_pixels.transpose());
    // PRX_DEBUG_VAR_1((pixels - computed_pixels).transpose());
    return pixels - computed_pixels;
  }

  void print(const std::string& s = "",
             const gtsam::KeyFormatter& keyFormatter = symbol_factory_t::formatter) const override
  {
    std::cout << s << "(camera_to_world_factor_t)  keys = { ";
    for (gtsam::Key key : keys())
    {
      std::cout << keyFormatter(key) << " ";
    }
    std::cout << "}" << std::endl;
  }
};

}  // namespace fg
}  // namespace prx