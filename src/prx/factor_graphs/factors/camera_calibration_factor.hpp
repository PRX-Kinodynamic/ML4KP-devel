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

class camera_projection_factor_t : public noise_model_1p2_factor_t<2, 3, 12>
{
  using Base = noise_model_1p2_factor_t<2, 3, 12>;
  using G = Eigen::Matrix<double, 2, 12>;

public:
  using WorldPosition = Base::X0;
  using Projection = Base::X1;
  using Pixel = Eigen::Vector2d;

  camera_projection_factor_t(gtsam::Key key_world_position, gtsam::Key key_projection, Pixel pixel,
                             const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(key_world_position, key_projection, cost_model), _pixel(pixel)
  {
  }

  virtual Pixel compute_error(const WorldPosition& position, const Projection& projection) const override
  {
    const double u{ _pixel[0] };
    const double v{ _pixel[1] };
    const Eigen::RowVector4d row_position{ position[0], position[1], position[2], 1 };
    G g{ G::Zero() };

    g.block<1, 4>(0, 0) = row_position;
    g.block<1, 4>(1, 4) = row_position;
    g.block<1, 4>(0, 8) = row_position * u;
    g.block<1, 4>(1, 8) = row_position * v;
    // PRX_DEBUG_VAR_1(g);
    return g * projection;
  }

  void print(const std::string& s = "",
             const gtsam::KeyFormatter& keyFormatter = symbol_factory_t::formatter) const override
  {
    std::cout << s << "(camera_projection_factor_t)  keys = { ";
    for (gtsam::Key key : keys())
    {
      std::cout << keyFormatter(key) << " ";
    }
    std::cout << "}" << std::endl;
  }

private:
  const Pixel _pixel;
};

class camera_projection_norm_factor_t : public noise_model_1p1_factor_t<1, 12>
{
  using Base = noise_model_1p1_factor_t<1, 12>;
  using G = Eigen::Matrix<double, 2, 12>;

public:
  using Norm = Base::Xerr;
  using Projection = Base::X0;

  camera_projection_norm_factor_t(gtsam::Key key_projection, const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(key_projection, cost_model)
  {
  }

  virtual Norm compute_error(const Projection& projection) const override
  {
    const double p31{ projection[8] };
    const double p32{ projection[9] };
    const double p33{ projection[10] };

    return Norm(1.0 - (p31 * p31 + p32 * p32 + p33 * p33));
  }

  void print(const std::string& s = "",
             const gtsam::KeyFormatter& keyFormatter = symbol_factory_t::formatter) const override
  {
    std::cout << s << "(camera_projection_norm_factor_t)  keys = { ";
    for (gtsam::Key key : keys())
    {
      std::cout << keyFormatter(key) << " ";
    }
    std::cout << "}" << std::endl;
  }

private:
};

class camera_phi_factor_t : public noise_model_3factor_t<3, 12, 1>
{
  using Base = noise_model_3factor_t<3, 12, 1>;
  using G = Eigen::Matrix<double, 2, 12>;

public:
  using Position = Base::X0;
  using Projection = Base::X1;
  using Scale = Base::X2;
  using Pixel = Eigen::Vector2d;
  using ProjectionMat = Eigen::Matrix<double, 3, 4>;

  camera_phi_factor_t(gtsam::Key position, gtsam::Key key_projection, gtsam::Key key_scale, const Pixel pixel,
                      const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(position, key_projection, key_scale, cost_model), _pixel(pixel)
  {
  }

  virtual Position compute_error(const Position& position, const Projection& p, const Scale& s) const override
  {
    const double scale{ s[0] };
    const Eigen::Vector3d m(_pixel[0], _pixel[1], 1);
    const Eigen::Vector4d M(position[0], position[1], position[2], 1);
    ProjectionMat Pr{ ProjectionMat::Zero() };
    projection_vector_to_matrix(p, Pr);
    // Pr.block<1, 4>(0, 0) = p.segment<4>(0);
    // Pr.block<1, 4>(1, 0) = p.segment<4>(4);
    // Pr.block<1, 4>(2, 0) = p.segment<4>(8);
    // PRX_DEBUG_VAR_1(Pr);
    // PRX_DEBUG_VAR_1(p);
    return scale * m - Pr * M;
  }

  static void projection_vector_to_matrix(const Projection& pvec, ProjectionMat& pmat)
  {
    pmat.block<1, 4>(0, 0) = pvec.segment<4>(0);
    pmat.block<1, 4>(1, 0) = pvec.segment<4>(4);
    pmat.block<1, 4>(2, 0) = pvec.segment<4>(8);
  }

  void print(const std::string& s = "",
             const gtsam::KeyFormatter& keyFormatter = symbol_factory_t::formatter) const override
  {
    std::cout << s << "(camera_phi_factor_t)  keys = { ";
    for (gtsam::Key key : keys())
    {
      std::cout << keyFormatter(key) << " ";
    }
    std::cout << "}" << std::endl;
  }

private:
  const Pixel _pixel;
};

class aruco_camera_projection_factor_t : public noise_model_6factor_t<2, 3, 3, 3, 3, 12>
{
  using Base = noise_model_6factor_t<2, 3, 3, 3, 3, 12>;
  using G = Eigen::Matrix<double, 2, 12>;

public:
  using Pixel = Base::X0;
  using Corner0 = Base::X1;
  using Corner1 = Base::X2;
  using Corner2 = Base::X3;
  using Corner3 = Base::X4;
  using Projection = Base::X5;

  aruco_camera_projection_factor_t(gtsam::Key key_pixel, gtsam::Key key_corner_0, gtsam::Key key_corner_1,
                                   gtsam::Key key_corner_2, gtsam::Key key_corner_3, gtsam::Key key_projection,
                                   const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(key_pixel, key_corner_0, key_corner_1, key_corner_2, key_corner_3, key_projection, cost_model)
  {
  }

  virtual Pixel compute_error(const Pixel& pixel, const Corner0& corner_0, const Corner1& corner_1,
                              const Corner2& corner_2, const Corner3& corner_3,
                              const Projection& projection) const override
  {
    const double u{ pixel[0] };
    const double v{ pixel[1] };
    const Eigen::Vector3d midpt{ (corner_0 + corner_1 + corner_2 + corner_3) / 4.0 };
    const Eigen::RowVector4d row_position{ midpt[0], midpt[1], midpt[2], 1 };
    G g{ G::Zero() };

    g.block<1, 4>(0, 0) = row_position;
    g.block<1, 4>(1, 4) = row_position;
    g.block<1, 4>(0, 8) = row_position * u;
    g.block<1, 4>(1, 8) = row_position * v;
    // PRX_DEBUG_VAR_1(g);
    return g * projection;
  }

  void print(const std::string& s = "",
             const gtsam::KeyFormatter& keyFormatter = symbol_factory_t::formatter) const override
  {
    std::cout << s << "(aruco_camera_projection_factor_t)  keys = { ";
    for (gtsam::Key key : keys())
    {
      std::cout << keyFormatter(key) << " ";
    }
    std::cout << "}" << std::endl;
  }
};

class projection_to_rotation_factor_t : public noise_model_2factor_t<9, 12>
{
  using Base = noise_model_2factor_t<9, 12>;

public:
  using RotationVec = Base::X0;
  using Projection = Base::X1;

  projection_to_rotation_factor_t(gtsam::Key key_rot_vec, gtsam::Key key_projection,
                                  const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(key_rot_vec, key_projection, cost_model)
  {
  }

  virtual RotationVec compute_error(const RotationVec& rot_vec, const Projection& projection) const override
  {
    const Eigen::Matrix<double, 3, 4> P{ projection.reshaped(3, 4) };
    const Eigen::Matrix3d B{ P.block<3, 3>(0, 0) };
    Eigen::Matrix3d K{ B * B.transpose() };
    K = K / K(2, 2);
    const double u0{ K(0, 2) };
    const double v0{ K(1, 2) };
    const double ku{ K(0, 0) };
    const double kc{ K(0, 1) };
    const double kv{ K(1, 1) };

    const double beta{ std::sqrt(ku - v0 * v0) };
    const double gamma{ (kc - u0 * v0) / beta };
    const double alpha{ std::sqrt(ku - u0 * u0 - gamma * gamma) };
    Eigen::Matrix3d A{ Eigen::Matrix3d::Zero() };
    A << alpha, gamma, u0,  // no-lint
        0, beta, v0,        // no-lint
        0, 0, 1;

    const Eigen::Matrix3d Rp{ A.inverse() * B };

    return Rp.reshaped(9, 1) - rot_vec;
  }

  void print(const std::string& s = "",
             const gtsam::KeyFormatter& keyFormatter = symbol_factory_t::formatter) const override
  {
    std::cout << s << "(projection_to_rotation_factor_t)  keys = { ";
    for (gtsam::Key key : keys())
    {
      std::cout << keyFormatter(key) << " ";
    }
    std::cout << "}" << std::endl;
  }
};

class projection_to_translation_factor_t : public noise_model_2factor_t<3, 12>
{
  using Base = noise_model_2factor_t<3, 12>;

public:
  using Translation = Base::X0;
  using Projection = Base::X1;

  projection_to_translation_factor_t(gtsam::Key key_translation, gtsam::Key key_projection,
                                     const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(key_translation, key_projection, cost_model)
  {
  }

  virtual Translation compute_error(const Translation& translation, const Projection& projection) const override
  {
    const Eigen::Matrix<double, 3, 4> P{ projection.reshaped(3, 4) };
    const Eigen::Matrix3d B{ P.block<3, 3>(0, 0) };
    const Eigen::Vector3d b{ P.block<3, 1>(0, 3) };
    Eigen::Matrix3d K{ B * B.transpose() };
    K = K / K(2, 2);
    const double u0{ K(0, 2) };
    const double v0{ K(1, 2) };
    const double ku{ K(0, 0) };
    const double kc{ K(0, 1) };
    const double kv{ K(1, 1) };

    const double beta{ std::sqrt(ku - v0 * v0) };
    const double gamma{ (kc - u0 * v0) / beta };
    const double alpha{ std::sqrt(ku - u0 * u0 - gamma * gamma) };
    Eigen::Matrix3d A{ Eigen::Matrix3d::Zero() };
    A << alpha, gamma, u0,  // no-lint
        0, beta, v0,        // no-lint
        0, 0, 1;

    const Eigen::Vector3d tp{ A.inverse() * b };

    return tp - translation;
  }

  void print(const std::string& s = "",
             const gtsam::KeyFormatter& keyFormatter = symbol_factory_t::formatter) const override
  {
    std::cout << s << "(projection_to_translation_factor_t)  keys = { ";
    for (gtsam::Key key : keys())
    {
      std::cout << keyFormatter(key) << " ";
    }
    std::cout << "}" << std::endl;
  }
};

}  // namespace fg
}  // namespace prx