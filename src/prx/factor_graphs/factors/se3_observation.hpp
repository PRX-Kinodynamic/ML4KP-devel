#pragma once
#include <array>
#include <numeric>
#include <gtsam/config.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "prx/factor_graphs/lie_groups/se3.hpp"
#include "prx/factor_graphs/utilities/symbols_factory.hpp"

namespace prx
{
namespace fg
{
class SE3_observation_factor_t : public gtsam::NoiseModelFactor1<se3_t>
{
public:
  using SE3 = prx::fg::se3_t;
  using Translation = Eigen::Vector<double, 3>;
  using Rotation = Eigen::Matrix<double, 3, 3>;
  using SkewMatrix = Eigen::Matrix<double, 3, 3>;
  using Base = gtsam::NoiseModelFactor1<SE3>;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;
  using Jacobian = Eigen::Matrix<double, 3, 6>;

  // using Vector = Eigen::Vector<double, Dim>;

  SE3_observation_factor_t(const gtsam::Key key, const Translation offset, const Translation z,
                           const NoiseModel& cost_model)
    : Base(cost_model, key)
    , _offset(offset)
    , _z(z)
    , _Hzero(Jacobian::Zero())
    , _skew_offset(gtsam::skewSymmetric(-offset[0], -offset[1], -offset[2]))
  {
  }

  static Translation predict(const SE3& x, const Translation& offset)
  {
    return x * offset;
  }

  virtual Eigen::VectorXd evaluateError(const SE3& x, boost::optional<Eigen::MatrixXd&> H0 = boost::none) const override
  {
    const Translation pt_expected{ predict(x, _offset) };
    const Translation error{ pt_expected - _z };

    if (H0)
    {
      const Rotation R{ x.rotation_matrix() };
      *H0 = _Hzero;
      H0->leftCols<3>() = R * _skew_offset;
      H0->rightCols<3>() = R;
    }
    return error;
  }

  void print(const std::string& s,
             const gtsam::KeyFormatter& keyFormatter = prx::fg::symbol_factory_t::formatter) const override
  {
    const std::string key_x{ keyFormatter(this->template key<1>()) };

    std::cout << "SE3 Observation Factor: [" << key_x << "]\n";
    std::cout << "\t Offset: " << _offset.transpose() << "\n";
    std::cout << "\t Observation: " << _z.transpose() << "\n";
  }

private:
  Translation _offset;
  Translation _z;
  const Jacobian _Hzero;
  const SkewMatrix _skew_offset;
};

}  // namespace fg
}  // namespace prx