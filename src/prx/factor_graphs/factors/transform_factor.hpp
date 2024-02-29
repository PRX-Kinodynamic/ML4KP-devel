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

class transform_factor_t : public gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Pose3, gtsam::Pose3>
{
public:
  using Base = gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Pose3, gtsam::Pose3>;
  using Jacobian = Eigen::MatrixXd;
  using Error = gtsam::Vector;
  transform_factor_t(){};
  transform_factor_t(const gtsam::Key& pi, const gtsam::Key& pj, const gtsam::Key& Tij,
                     const gtsam::noiseModel::Base::shared_ptr& cost_model)
    : Base(cost_model, pi, pj, Tij)
  {
  }

  Error evaluateError(const gtsam::Pose3& pi, const gtsam::Pose3& pj, const gtsam::Pose3& Tij,
                      boost::optional<Jacobian&> H1 = boost::none, boost::optional<Jacobian&> H2 = boost::none,
                      boost::optional<Jacobian&> H3 = boost::none) const override
  {
    const gtsam::Pose3 pj_predicted{ Tij.transformPoseFrom(pi, H1 ? &D_predict_pi : 0, H3 ? &D_predict_Tij : 0) };
    const gtsam::Pose3 error{ pj.between(pj_predicted, H2 ? &D_error_pj : 0, H1 || H3 ? &D_error_predict : 0) };

    const Eigen::Vector<double, 3> p_error{ error.translation() };
    const Eigen::Vector<double, 3> R_error{ gtsam::Rot3::Logmap(error.rotation()) };
    if (H1)
    {
      *H1 = D_error_predict * D_predict_pi;
    }
    if (H2)
    {
      *H2 = D_error_pj;
    }
    if (H3)
    {
      *H3 = D_error_predict * D_predict_Tij;
    }
    return (Eigen::Vector<double, 6>() << R_error, p_error).finished();
  }

  static std::string to_string(const Eigen::Vector3d t)
  {
    std::stringstream strstr;
    strstr << t[0] << " ";
    strstr << t[1] << " ";
    strstr << t[2] << " ";
    return strstr.str();
  }
  static std::string to_string(const Eigen::Quaterniond q)
  {
    std::stringstream strstr;
    strstr << q.w() << " ";
    strstr << q.x() << " ";
    strstr << q.y() << " ";
    strstr << q.z() << " ";
    return strstr.str();
  }

  void eval_to_stream(gtsam::Values& values, std::ostream& os)
  {
    const gtsam::Pose3 pi{ values.at<gtsam::Pose3>(key<1>()) };
    const gtsam::Pose3 pj{ values.at<gtsam::Pose3>(key<2>()) };
    const gtsam::Pose3 Tij{ values.at<gtsam::Pose3>(key<3>()) };

    const Eigen::Vector3d ti{ pi.translation().transpose() };
    const Eigen::Vector3d tj{ pj.translation().transpose() };
    const Eigen::Vector3d tij{ Tij.translation().transpose() };

    const Eigen::Quaterniond qi{ pi.rotation().toQuaternion() };
    const Eigen::Quaterniond qj{ pj.rotation().toQuaternion() };
    const Eigen::Quaterniond qij{ Tij.rotation().toQuaternion() };

    os << to_string(ti) << to_string(qi) << " ";    // 2, 3, 4, 5
    os << to_string(tj) << to_string(qj) << " ";    // 2, 3, 4, 5
    os << to_string(tij) << to_string(qij) << " ";  // 2, 3, 4, 5
    os << "\n";
  }

private:
  mutable Eigen::Matrix<double, 6, 6> D_predict_pi;
  mutable Eigen::Matrix<double, 6, 6> D_predict_Tij;
  mutable Eigen::Matrix<double, 6, 6> D_error_pj, D_error_predict;
};
}  // namespace fg
}  // namespace prx