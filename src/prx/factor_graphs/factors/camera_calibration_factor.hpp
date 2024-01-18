#pragma once
#include <array>
#include <numeric>
#include <gtsam/config.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "prx/utilities/math/first_order_derivative.hpp"
#include "prx/factor_graphs/factors/noise_model_factor.hpp"
#include <prx/factor_graphs/utilities/symbols_factory.hpp>
// #include "prx/factor_graphs/factors/noise_model_factor.hpp"
// #include "prx/factor_graphs/utilities/perception/camera.hpp"
// #include "prx/factor_graphs/utilities/symbols_factory.hpp"

namespace prx
{
namespace fg
{

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

}  // namespace fg
}  // namespace prx