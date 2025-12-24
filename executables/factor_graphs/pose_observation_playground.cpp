#include <iostream>

#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/type_conversions.hpp"
#include "prx/factor_graphs/lie_groups/se3.hpp"
#include "prx/factor_graphs/factors/se3_observation.hpp"

#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/rrt_star.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/planners/planner.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/simulation/collision_checking/collision_group.hpp"

#include "prx/simulation/plants/first_order_free_body.hpp"
#include "prx/factor_graphs/factors/obstacle_factor.hpp"
#include "prx/factor_graphs/utilities/default_parameters.hpp"
#include "prx/factor_graphs/lie_groups/screw_axis.hpp"
#include "prx/factor_graphs/lie_groups/lie_integrator.hpp"
#include "prx/factor_graphs/factors/screw_smoothing.hpp"
#include "prx/factor_graphs/utilities/values_utilities.hpp"
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/basis/FitBasis.h>
#include <gtsam/basis/Chebyshev2.h>

// using prx::utilities::convert_to;
using Rotation = gtsam::Rot3;
using Translation = Eigen::Vector<double, 3>;
using SE3 = gtsam::Pose3;
using Values = gtsam::Values;
using Graph = gtsam::NonlinearFactorGraph;
// template <typename SE3>
class SE3_symmetric_observation_factor_t : public gtsam::NoiseModelFactor1<gtsam::Pose3>
{
public:
  using SE3 = gtsam::Pose3;
  using Translation = Eigen::Vector<double, 3>;
  using Rotation = gtsam::Rot3;
  using SkewMatrix = Eigen::Matrix<double, 3, 3>;
  using Base = gtsam::NoiseModelFactor1<SE3>;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;
  using Jacobian = Eigen::Matrix<double, 3, 6>;

  // using Vector = Eigen::Vector<double, Dim>;

  template <typename RotationType>
  SE3_symmetric_observation_factor_t(const gtsam::Key key, const RotationType offset, const Translation zA,
                                     const Translation zB, const NoiseModel& cost_model)
    : Base(cost_model, key), _R_offset_inv(Rotation(offset).inverse()), _zA(zA), _zB(zB)
  {
  }

  static Eigen::VectorXd compute_error(const SE3& x, const Translation& zA, const Translation& zB,
                                       const Rotation& Roffset_inv, boost::optional<Eigen::MatrixXd&> H0 = boost::none)
  {
    // const Translation  x.transformFrom(_zA, Hself = boost::none,
    //                             OptionalJacobian<3, 3> Hpoint = boost::none) const;

    // pRZ.transformFrom(p0i.transformFrom(zB)) - p0i.transformFrom(zA)

    Eigen::MatrixXd xinv_H_x;
    Eigen::MatrixXd pA_H_xinv;
    Eigen::MatrixXd pB_H_xinv;
    Eigen::MatrixXd pbRot_H_pB;
    // Eigen::MatrixXd Rx_H_x;
    // Eigen::MatrixXd Rxinv_H_Rx;
    // Eigen::MatrixXd pBrot_H_pB;
    const SE3 xinv{ x.inverse(xinv_H_x) };
    const Translation pA{ xinv.transformFrom(zA, pA_H_xinv) };
    const Translation pB{ xinv.transformFrom(zB, pB_H_xinv) };
    const Translation pBRot{ Roffset_inv.rotate(pB, boost::none, pbRot_H_pB) };
    const Translation error{ pA - pBRot };
    PRX_DBG_VARS(x);
    PRX_DBG_VARS(Roffset_inv);
    PRX_DBG_VARS(zA.transpose(), zB.transpose());
    PRX_DBG_VARS(pA.transpose(), pBRot.transpose());
    // PRX_DBG_VARS(pBrot.transpose());
    PRX_DBG_VARS(error.transpose());

    if (H0)
    {
      const Eigen::Matrix3d err_H_pA{ Eigen::Matrix3d::Identity() };
      const Eigen::Matrix3d err_H_pBRot{ -Eigen::Matrix3d::Identity() };
      *H0 = err_H_pA * pA_H_xinv * xinv_H_x +  // no-lint
            err_H_pBRot * pbRot_H_pB * pB_H_xinv * xinv_H_x;
      // err_H_pA = I
      // *H0 = /* err_H_pA */ pA_H_xinv * xinv_H_x +  // no-lint
      //       err_H_pB * pBrot_H_pB * pB_H_xinv * xinv_H_x;
    }
    return error;
  }

  virtual Eigen::VectorXd evaluateError(const SE3& x, boost::optional<Eigen::MatrixXd&> H0 = boost::none) const override
  {
    const Eigen::VectorXd error{ compute_error(x, _zA, _zB, _R_offset_inv, H0) };

    return error;
  }

private:
  Rotation _R_offset_inv;
  Translation _zA;
  Translation _zB;
  // const Jacobian _Hzero;
  // const SkewMatrix _skew_offset;
};

class SE3_single_observation_factor_t : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Rot3>
{
public:
  using SE3 = gtsam::Pose3;
  using Translation = Eigen::Vector<double, 3>;
  using Rotation = gtsam::Rot3;
  using SkewMatrix = Eigen::Matrix<double, 3, 3>;
  using Base = gtsam::NoiseModelFactor1<SE3, gtsam::Rot3>;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;
  using Jacobian = Eigen::Matrix<double, 3, 6>;

  // using Vector = Eigen::Vector<double, Dim>;

  SE3_single_observation_factor_t(const gtsam::Key key_se3, const gtsam::Key key_offset, const Translation zi,
                                  const Translation offset, const NoiseModel& cost_model)
    : Base(cost_model, key_se3, key_offset), _zi(zi), _offset(offset)
  {
  }

  static Translation prediction(const SE3& x, const Rotation& Roffset, const Translation& offset,  // no-lint
                                boost::optional<Eigen::MatrixXd&> Hx = boost::none,
                                boost::optional<Eigen::MatrixXd&> HR = boost::none)
  {
    Eigen::MatrixXd rOff_H_off;
    Eigen::MatrixXd pred_H_x;
    Eigen::MatrixXd pred_H_rOff;

    const Translation rot_offset{ Roffset.rotate(offset, rOff_H_off) };
    const Translation pred{ x.transformFrom(rot_offset, pred_H_x, pred_H_rOff) };

    if (Hx)
    {
      *Hx = pred_H_x;
    }
    if (HR)
    {
      *HR = pred_H_rOff * rOff_H_off;
    }
    return pred;
  }

  static Eigen::VectorXd compute_error(const SE3& x, const Rotation& Roffset, const Translation& zi,
                                       const Translation& offset,  // no-lint
                                       boost::optional<Eigen::MatrixXd&> Hx = boost::none,
                                       boost::optional<Eigen::MatrixXd&> HR = boost::none)
  {
    // const Translation  x.transformFrom(_zA, Hself = boost::none,
    //                             OptionalJacobian<3, 3> Hpoint = boost::none) const;

    // pRZ.transformFrom(p0i.transformFrom(zB)) - p0i.transformFrom(zA)

    Eigen::MatrixXd pred_H_x;
    Eigen::MatrixXd pred_H_Roff;
    // Eigen::MatrixXd pB_H_xinv;
    // Eigen::MatrixXd pbRot_H_pB;

    const Translation pred{ prediction(x, Roffset, offset, pred_H_x, pred_H_Roff) };  // no-lint
    const Translation err{ pred - zi };

    if (Hx)
    {
      const Eigen::Matrix3d err_H_pred{ Eigen::Matrix3d::Identity() };
      *Hx = err_H_pred * pred_H_x;
    }
    if (HR)
    {
      const Eigen::Matrix3d err_H_pred{ Eigen::Matrix3d::Identity() };
      // const Eigen::Matrix3d err_H_zi{ -Eigen::Matrix3d::Identity() };
      *HR = err_H_pred * pred_H_Roff;
    }
    return err;
  }

  virtual Eigen::VectorXd evaluateError(const SE3& x, const Rotation& Roffset,
                                        boost::optional<Eigen::MatrixXd&> Hx = boost::none,
                                        boost::optional<Eigen::MatrixXd&> HR = boost::none) const override
  {
    const Eigen::VectorXd error{ compute_error(x, Roffset, _zi, _offset, Hx, HR) };

    return error;
  }

private:
  Translation _offset;
  Translation _zi;
};

class rotation_symmetric_factor_t : public gtsam::NoiseModelFactor1<gtsam::Rot3, gtsam::Rot3>
{
public:
  using SE3 = gtsam::Pose3;
  using Translation = Eigen::Vector<double, 3>;
  using Rotation = gtsam::Rot3;
  using SkewMatrix = Eigen::Matrix<double, 3, 3>;
  using Base = gtsam::NoiseModelFactorN<Rotation, Rotation>;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;
  using Jacobian = Eigen::Matrix<double, 3, 6>;

  // using Vector = Eigen::Vector<double, Dim>;

  template <typename RotationType>
  rotation_symmetric_factor_t(const gtsam::Key key_Ri, const gtsam::Key key_Rk, const RotationType offset,
                              const NoiseModel& cost_model)
    : Base(cost_model, key_Ri, key_Rk), _offset(offset)
  {
  }

  static Eigen::VectorXd compute_error(const Rotation& Ri, const Rotation& Rk, const Rotation& Roff,
                                       boost::optional<Eigen::MatrixXd&> HRi = boost::none,
                                       boost::optional<Eigen::MatrixXd&> HRk = boost::none)
  {
    Eigen::MatrixXd Rkoff_H_Rk;
    Eigen::MatrixXd RkoffInv_H_Rkoff;
    Eigen::MatrixXd RikInv_H_Rkoff;
    Eigen::MatrixXd RikInv_H_Ri;
    Eigen::MatrixXd err_H_RikInv;

    const Rotation Rkoff{ gtsam::traits<Rotation>::Compose(Rk, Roff, Rkoff_H_Rk) };
    const Rotation Rkoff_inv{ Rkoff.inverse(RkoffInv_H_Rkoff) };
    const Rotation RikInv{ gtsam::traits<Rotation>::Compose(Rkoff_inv, Ri, RikInv_H_Rkoff, RikInv_H_Ri) };
    const Eigen::Vector3d err{ Rotation::Logmap(RikInv, err_H_RikInv) };

    if (HRi)
    {
      *HRi = err_H_RikInv * RikInv_H_Ri;
    }
    if (HRk)
    {
      // const Eigen::Matrix3d err_H_zi{ -Eigen::Matrix3d::Identity() };
      *HRk = err_H_RikInv * RikInv_H_Rkoff * RkoffInv_H_Rkoff * Rkoff_H_Rk;
    }
    return err;
  }

  virtual Eigen::VectorXd evaluateError(const Rotation& Ri, const Rotation& Rk,
                                        boost::optional<Eigen::MatrixXd&> HRi = boost::none,
                                        boost::optional<Eigen::MatrixXd&> HRk = boost::none) const override
  {
    const Eigen::VectorXd error{ compute_error(Ri, Rk, _offset, HRi, HRk) };

    return error;
  }

private:
  Rotation _offset;
};

class bar_two_observations_factor_t : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Rot3>
{
public:
  using SE3 = gtsam::Pose3;
  using Translation = Eigen::Vector<double, 3>;
  using Rotation = gtsam::Rot3;
  using SkewMatrix = Eigen::Matrix<double, 3, 3>;
  using Base = gtsam::NoiseModelFactor1<SE3, gtsam::Rot3>;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;
  using Jacobian = Eigen::Matrix<double, 3, 6>;

  enum ObservationIdx
  {
    First = 0,
    Second
  };
  // using Vector = Eigen::Vector<double, Dim>;

  template <typename RotationType>
  bar_two_observations_factor_t(const gtsam::Key key_se3, const gtsam::Key key_offset, const Translation zi,
                                const Translation offset, const RotationType Roffset,
                                const ObservationIdx observation_id, const NoiseModel& cost_model)
    : Base(cost_model, key_se3, key_offset), _zi(zi), _offset(offset), _observation_id(observation_id), _Roff(Roffset)
  {
  }

  static Translation prediction(const SE3& x, const Rotation& Rx, const Rotation& Roff, const Translation& offset,
                                const ObservationIdx& observation_id,  // no-lint
                                boost::optional<Eigen::MatrixXd&> Hx = boost::none,
                                boost::optional<Eigen::MatrixXd&> HR = boost::none)
  {
    Eigen::MatrixXd rOff_H_r;
    Eigen::MatrixXd pred_H_x;
    Eigen::MatrixXd pred_H_rOff;
    Eigen::Matrix3d r_H_Rx{ Eigen::Matrix3d::Identity() };

    Rotation rotation{ Rx };
    if (observation_id == ObservationIdx::First)
    {
      rotation = gtsam::traits<Rotation>::Compose(Rx, Roff, r_H_Rx);
    }

    const Translation rot_offset{ rotation.rotate(offset, rOff_H_r) };
    const Translation pred{ x.transformFrom(rot_offset, pred_H_x, pred_H_rOff) };

    if (Hx)
    {
      *Hx = pred_H_x;
    }
    if (HR)
    {
      *HR = pred_H_rOff * rOff_H_r * r_H_Rx;
    }
    return pred;
  }

  static Eigen::VectorXd compute_error(const SE3& x, const Rotation& Rx, const Rotation& Roff, const Translation& zi,
                                       const Translation& offset, const ObservationIdx& observation_id,  // no-lint
                                       boost::optional<Eigen::MatrixXd&> Hx = boost::none,
                                       boost::optional<Eigen::MatrixXd&> HR = boost::none)
  {
    Eigen::MatrixXd pred_H_x;
    Eigen::MatrixXd pred_H_Roff;
    // Eigen::MatrixXd pB_H_xinv;
    // Eigen::MatrixXd pbRot_H_pB;

    const Translation pred{ prediction(x, Rx, Roff, offset, observation_id, pred_H_x, pred_H_Roff) };  // no-lint
    const Translation err{ pred - zi };

    if (Hx)
    {
      const Eigen::Matrix3d err_H_pred{ Eigen::Matrix3d::Identity() };
      *Hx = err_H_pred * pred_H_x;
    }
    if (HR)
    {
      const Eigen::Matrix3d err_H_pred{ Eigen::Matrix3d::Identity() };
      // const Eigen::Matrix3d err_H_zi{ -Eigen::Matrix3d::Identity() };
      *HR = err_H_pred * pred_H_Roff;
    }
    return err;
  }

  virtual Eigen::VectorXd evaluateError(const SE3& x, const Rotation& Rx,
                                        boost::optional<Eigen::MatrixXd&> Hx = boost::none,
                                        boost::optional<Eigen::MatrixXd&> HR = boost::none) const override
  {
    // compute_error(const SE3& x, const Rotation& Rx, const Rotation& Roff, const Translation& zi,
    //                                    const Translation& offset,
    const Eigen::VectorXd error{ compute_error(x, Rx, _Roff, _zi, _offset, _observation_id, Hx, HR) };

    return error;
  }

private:
  const Translation _offset;
  const Translation _zi;
  const Rotation _Roff;
  const ObservationIdx _observation_id;
};

const Translation noise(const double mu, const double sigma)
{
  const double x{ prx::gaussian_random(mu, sigma) };
  const double y{ prx::gaussian_random(mu, sigma) };
  const double z{ prx::gaussian_random(mu, sigma) };
  return { x, y, z };
}

int main(int argc, char* argv[])
{
  prx::param_loader params{};
  // params["print_matrix"].set(false);

  params.add_opts(argc, argv);

  const Translation zero{ Translation::Zero() };
  const Translation offset{ 0.325 / 2.0, 0.0, 0.0 };
  const Rotation rot_offset{ 0.0, 0.0, 0.0, 1.0 };
  const SE3 pose_offset{ rot_offset, zero };
  const SE3 pose_zero{ Rotation(), zero };

  const Translation t0{ 3.0, 3.0, 3.0 };
  const Rotation r0{ 1.0, 0.0, 0.0, 0.0 };

  const SE3 p0{ r0, t0 };

  const double mu{ 0.0 };
  const double sigma{ 0.01 };

  // PRX_DBG_VARS(epsA.transpose());
  // PRX_DBG_VARS(epsB.transpose());

  // PRX_DBG_VARS(endcapA.transpose());
  // PRX_DBG_VARS(endcapB.transpose());

  // SE3_symmetric_observation_factor_t::compute_error(p0, zB, zA, rot_offset);

  const gtsam::Key keyX(0);
  const gtsam::Key keyRi(1);
  const gtsam::Key keyRk(2);

  Graph graph;
  Values values;

  // values.insert(keyX, p0);
  values.insert(keyX, pose_zero);
  values.insert(keyRi, Rotation());
  values.insert(keyRk, Rotation());
  // gtsam::noiseModel::Base::shared_ptr se3_noise{ gtsam::noiseModel::Isotropic::Sigma(3, sigma) };

  const gtsam::Rot3 Roff(0, 0, 1, 0);
  graph.emplace_shared<rotation_symmetric_factor_t>(keyRi, keyRk, rot_offset, nullptr);
  for (int i = 0; i < 10; ++i)
  {
    const Translation epsA(noise(mu, sigma));
    const Translation epsB(noise(mu, sigma));

    const Translation endcapA{ p0 * offset };
    const Translation endcapB{ p0 * -offset };

    const Translation zA{ endcapA + epsA };
    const Translation zB{ endcapB + epsB };

    // SE3_symmetric_observation_factor_t::compute_error(p0, zA, zB, rot_offset);
    // if (i % 2)
    // rotation_symmetric_factor_t(const gtsam::Key key_Ri, const gtsam::Key key_Rk, const RotationType offset,
    // SE3_single_observation_factor_t(const gtsam::Key key_se3, const gtsam::Key key_offset, const Translation zi,
    //                                const Translation offset, const NoiseModel& cost_model)

    using BarEstimationFactor = bar_two_observations_factor_t;
    graph.emplace_shared<BarEstimationFactor>(keyX, keyRi, zA, offset, Roff, BarEstimationFactor::ObservationIdx::First,
                                              nullptr);
    graph.emplace_shared<BarEstimationFactor>(keyX, keyRi, zB, offset, Roff,
                                              BarEstimationFactor::ObservationIdx::Second, nullptr);

    // graph.emplace_shared<SE3_single_observation_factor_t>(keyX, keyRi, zA, offset, nullptr);
    // graph.emplace_shared<SE3_single_observation_factor_t>(keyX, keyRk, zB, offset, nullptr);

    // graph.emplace_shared<SE3_symmetric_observation_factor_t>(keyX, rot_offset, zA, zB, nullptr);
    // else
    // graph.emplace_shared<SE3_symmetric_observation_factor_t>(keyX, rot_offset, zB, zA, nullptr);
  }

  gtsam::LevenbergMarquardtParams lm_params{ prx::fg::default_levenberg_marquardt_parameters() };
  lm_params.setUseFixedLambdaFactor(true);
  // lm_params.setMaxIterations(1);
  lm_params.setMaxIterations(100);

  gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);

  gtsam::Values result{ optimizer.optimize() };

  const SE3 Xres{ result.at<SE3>(keyX) };
  const Rotation RotRes_i{ result.at<Rotation>(keyRi) };
  // const Rotation RotRes_k{ result.at<Rotation>(keyRk) };

  PRX_DBG_VARS(RotRes_i);
  // PRX_DBG_VARS(RotRes_k);
  PRX_DBG_VARS(p0);
  PRX_DBG_VARS(Xres);

  return 0;
}
