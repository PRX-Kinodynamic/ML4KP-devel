#pragma once
#include <array>
#include <numeric>
#include <gtsam/config.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace prx
{
namespace fg
{
class screw_smoothing_factor_t : public noise_model_2factor_t<screw_axis_t, screw_axis_t>
{
  using Screw = prx::fg::screw_axis_t;
  using Rotation = Eigen::Matrix<double, 3, 3>;
  using Base = noise_model_2factor_t<Screw, Screw>;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;
  using Jacobian = Eigen::Matrix<double, 2, 6>;
  using Vector3 = Eigen::Vector3d;
  using Vector2 = Eigen::Vector2d;

public:
  screw_smoothing_factor_t(const gtsam::Key s0_key, const gtsam::Key s1_key, const double Lwv,
                           const NoiseModel& cost_model)
    : screw_smoothing_factor_t(s0_key, s1_key, Lwv, Lwv, cost_model) {};

  screw_smoothing_factor_t(const gtsam::Key s0_key, const gtsam::Key s1_key, const double Lw, const double Lv,
                           const NoiseModel& cost_model)
    : Base(s0_key, s1_key, cost_model, _epsilon), _L_cts(Lw, Lw, Lw, Lv, Lv, Lv)
  {
  }

  virtual Screw predict(const Screw& s) const override
  {
    return s;
  }

  virtual bool active(const gtsam::Values& values) const override
  {
    const Screw s0{ values.at<Screw>(this->template key<1>()) };
    const Screw s1{ values.at<Screw>(this->template key<2>()) };
    const Screw expected{ s0 - s1 };
    const bool is_active_w{ expected[0] > _L_cts[0] or expected[1] > _L_cts[1] or expected[2] > _L_cts[2] };
    const bool is_active_v{ expected[3] > _L_cts[3] or expected[4] > _L_cts[4] or expected[5] > _L_cts[5] };
    return is_active_w or is_active_v;
  }

  virtual Eigen::Vector<double, 6> compute_error(const Screw& s0, const Screw& s1) const override
  {
    const Eigen::Vector<double, 6> diff{ (s1 - s0).vector() };

    const Eigen::Array<double, 6, 1> x_array{ diff.array() };
    const auto greater_than_L{ x_array > _L_cts };
    const Eigen::Vector<double, 6> error{ x_array * greater_than_L.template cast<double>() };
    // PRX_DEBUG_VAR_1(error)
    return error;
  }

private:
  const Eigen::Array<double, 6, 1> _L_cts;
  static constexpr double _epsilon{ 0.001 };
};

}  // namespace fg
}  // namespace prx