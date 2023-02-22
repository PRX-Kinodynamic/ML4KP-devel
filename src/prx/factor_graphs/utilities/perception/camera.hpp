#pragma once
#include "prx/utilities/defs.hpp"

/**
 *	Using this: https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#zhang2000
 *  As the main source
 **/
//
//
namespace prx
{
namespace fg
{
namespace camera
{
using Camera = Eigen::Vector<double, 5>;
using CameraMatrix = Eigen::Matrix3d;
using DistortionVector = Eigen::Vector<double, 8>;           // k1,k2,k3,k4,k5,k6,p1,p2
using DistortionVector4 = Eigen::Vector<double, 4>;          // k1,k2,p1,p2
using CameraWithDistortion4 = Eigen::Vector<double, 5 + 4>;  // k1,k2,p1,p2
using CameraWithDistortion = Eigen::Vector<double, 5 + 8>;   // k1,k2,k3,k4,k5,k6,p1,p2

using Pixel = Eigen::Vector2d;
using Position = Eigen::Vector3d;

// Camera matrix:
// [fx skew cx]
// [0   fy  cy]
// [0    0   1]
inline double get_fx(const Camera& camera)
{
  return camera[0];
}
inline double get_cx(const Camera& camera)
{
  return camera[1];
}
inline double get_fy(const Camera& camera)
{
  return camera[2];
}
inline double get_cy(const Camera& camera)
{
  return camera[3];
}
inline double get_skew(const Camera& camera)
{
  return camera[4];
}

static CameraMatrix get_camera_matrix(const Camera& camera)
{
  const double fx{ get_fx(camera) };
  const double skew{ get_skew(camera) };
  const double cx{ get_cx(camera) };
  const double fy{ get_fy(camera) };
  const double cy{ get_cy(camera) };

  CameraMatrix k;
  k << fx, skew, cx,  // no-lint
      0, fy, cy,      // no-lint
      0, 0, 1;        // no-lint
  return k;
};
static CameraMatrix get_camera_matrix(const CameraWithDistortion4& camera)
{
  return get_camera_matrix(Camera(camera.head(5)));
}
static DistortionVector4 get_distortion_vector(const CameraWithDistortion4& camera)
{
  return camera.tail(4);
}
inline double get_k1(const DistortionVector& distortion_vector)
{
  return distortion_vector[0];
}
inline double get_k2(const DistortionVector& distortion_vector)
{
  return distortion_vector[1];
}
inline double get_k3(const DistortionVector& distortion_vector)
{
  return distortion_vector[2];
}
inline double get_k4(const DistortionVector& distortion_vector)
{
  return distortion_vector[3];
}
inline double get_k5(const DistortionVector& distortion_vector)
{
  return distortion_vector[4];
}
inline double get_k6(const DistortionVector& distortion_vector)
{
  return distortion_vector[5];
}
inline double get_p1(const DistortionVector& distortion_vector)
{
  return distortion_vector[6];
}
inline double get_p2(const DistortionVector& distortion_vector)
{
  return distortion_vector[7];
}

Pixel real_lense_model(const Position& p, const Camera& camera, const DistortionVector& distortion_vector)
{
  const double fx{ get_fx(camera) };
  const double cx{ get_cx(camera) };
  const double fy{ get_fy(camera) };
  const double cy{ get_cy(camera) };
  const double skew{ get_skew(camera) };

  const double k1{ get_k1(distortion_vector) };
  const double k2{ get_k2(distortion_vector) };
  const double k3{ get_k3(distortion_vector) };
  const double k4{ get_k4(distortion_vector) };
  const double k5{ get_k5(distortion_vector) };
  const double k6{ get_k6(distortion_vector) };
  const double p1{ get_p1(distortion_vector) };
  const double p2{ get_p2(distortion_vector) };

  const double x{ p[0] };
  const double y{ p[1] };
  const double z{ p[2] };

  const double xp{ x / z };
  const double yp{ y / z };

  const double r{ std::pow(x, 2) + std::pow(y, 2) };
  const double distortion_n{ 1 + k1 * std::pow(r, 2) + k2 * std::pow(r, 4) + k3 * std::pow(r, 6) };
  const double distortion_d{ 1 + k4 * std::pow(r, 2) + k5 * std::pow(r, 4) + k6 * std::pow(r, 6) };
  const double distortion{ distortion_n / distortion_d };

  const double xpp{ xp * distortion + 2 * p1 * xp * yp + p2 * (std::pow(r, 2) + 2 * std::pow(xp, 2)) };
  const double ypp{ yp * distortion + p1 * (std::pow(r, 2) + 2 * std::pow(yp, 2)) + 2 * p2 * xp * yp };

  const double u{ (fx * xpp + cx) };
  const double v{ (fy * ypp + cy) };

  return Pixel(u, v);
}

Pixel real_lense_model(const Position& p, const CameraWithDistortion4& camera)
{
  CameraWithDistortion camera_ext;
  camera_ext << camera, 0, 0, 0, 0;
  return real_lense_model(p, camera_ext.head(5), camera_ext.tail(8));
}
Pixel real_lense_model(const Position& p, const CameraWithDistortion& camera)
{
  return real_lense_model(p, camera.head(5), camera.tail(8));
}

}  // namespace camera
}  // namespace fg
}  // namespace prx