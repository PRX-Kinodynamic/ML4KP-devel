#pragma once
#include <array>
#include <numeric>
#include <gtsam/config.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "prx/external/aruco_nano.h"

#include "prx/utilities/math/math_functions.hpp"
#include "prx/utilities/math/first_order_derivative.hpp"

#include "prx/factor_graphs/factors/noise_model_factor.hpp"
#include "prx/factor_graphs/utilities/perception/camera.hpp"

namespace prx
{
namespace fg
{

/**
 * @brief
 *
 * @tparam     DIM_0  Dimension of x0
 * @tparam     DIM_1  Dimension of x1
 */

class aruco_marker_translation_factor_t : public noise_model_6factor_t<3, 2, 2, 2, 2, 9>
{
  using Base = noise_model_6factor_t<3, 2, 2, 2, 2, 9>;

public:
  using Translation = typename Base::X0;
  using Pixel = typename Base::X1;
  using CameraWithDistortion = typename Base::X5;

  aruco_marker_translation_factor_t(gtsam::Key translation_key, gtsam::Key feature0_key, gtsam::Key feature1_key,
                                    gtsam::Key feature2_key, gtsam::Key feature3_key, gtsam::Key camera_key,
                                    double marker_size, const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(translation_key, feature0_key, feature1_key, feature2_key, feature3_key, camera_key, cost_model)
    , _marker_size(marker_size)
  {
  }

  virtual Translation compute_error(const Translation& t, const Pixel& f0, const Pixel& f1, const Pixel& f2,
                                    const Pixel& f3, const CameraWithDistortion& camera) const override
  {
    Eigen::Vector3d rotation;
    Eigen::Vector3d translation;
    prx::fg::camera::CameraMatrix camera_matrix{ prx::fg::camera::get_camera_matrix(camera) };
    prx::fg::camera::DistortionVector4 distortion_vector{ prx::fg::camera::get_distortion_vector(camera) };

    aruconano::Marker marker;
    marker.emplace_back(f0[0], f0[1]);
    marker.emplace_back(f1[0], f1[1]);
    marker.emplace_back(f2[0], f2[1]);
    marker.emplace_back(f3[0], f3[1]);
    std::tie(rotation, translation) = marker.estimatePoseEigen(camera_matrix, distortion_vector, _marker_size);
    return t - translation;
  }

private:
  double _marker_size;
};

class aruco_marker_rotation_factor_t : public noise_model_6factor_t<3, 2, 2, 2, 2, 9>
{
  using Base = noise_model_6factor_t<3, 2, 2, 2, 2, 9>;

public:
  using Rotation = typename Base::X0;
  using Pixel = typename Base::X1;
  using CameraWithDistortion = typename Base::X5;

  aruco_marker_rotation_factor_t(gtsam::Key rotation_key, gtsam::Key feature0_key, gtsam::Key feature1_key,
                                 gtsam::Key feature2_key, gtsam::Key feature3_key, gtsam::Key camera_key,
                                 double marker_size, const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(rotation_key, feature0_key, feature1_key, feature2_key, feature3_key, camera_key, cost_model)
    , _marker_size(marker_size)
  {
  }

  virtual Rotation compute_error(const Rotation& r, const Pixel& f0, const Pixel& f1, const Pixel& f2, const Pixel& f3,
                                 const CameraWithDistortion& camera) const override
  {
    Eigen::Vector3d rotation;
    Eigen::Vector3d translation;
    prx::fg::camera::CameraMatrix camera_matrix{ prx::fg::camera::get_camera_matrix(camera) };
    prx::fg::camera::DistortionVector4 distortion_vector{ prx::fg::camera::get_distortion_vector(camera) };

    aruconano::Marker marker;
    marker.emplace_back(f0[0], f0[1]);
    marker.emplace_back(f1[0], f1[1]);
    marker.emplace_back(f2[0], f2[1]);
    marker.emplace_back(f3[0], f3[1]);
    std::tie(rotation, translation) = marker.estimatePoseEigen(camera_matrix, distortion_vector, _marker_size);
    return r - rotation;
  }

private:
  double _marker_size;
};

}  // namespace fg
}  // namespace prx