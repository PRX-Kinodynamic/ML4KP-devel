#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>

#include <boost/algorithm/string/join.hpp>
#include <boost/optional.hpp>

#include "prx/simulation/plants/plants.hpp"
#include "prx/simulation/playback/trajectory.hpp"
#include "prx/simulation/system_factory.hpp"
#include "prx/simulation/system_group.hpp"
#include "prx/planning/world_model.hpp"

#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/csv_reader.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/visualization/three_js_group.hpp"

#include "prx/factor_graphs/lie_groups/se3.hpp"
#include "prx/factor_graphs/lie_groups/screw_axis.hpp"
#include "prx/factor_graphs/lie_groups/lie_integrator.hpp"
#include "prx/factor_graphs/utilities/symbols_factory.hpp"
#include "prx/factor_graphs/plants/pusher_slider.hpp"
#include "prx/factor_graphs/utilities/values_utilities.hpp"
#include "prx/factor_graphs/utilities/default_parameters.hpp"

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/Marginals.h>

using SE3 = prx::fg::se3_t;
using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

// a_T_b * b_T_c = a_T_c;
// a_T_b ~ NOT Known
// b_T_c ~ Known
// a_T_c ~ Known
class compose_factor_t : public gtsam::NoiseModelFactor1<SE3>
{
  using Base = gtsam::NoiseModelFactor1<SE3>;

public:
  compose_factor_t(const gtsam::Key& aTbKey, const SE3 bTc, const SE3 aTc, const NoiseModel& cost_model = nullptr)
    : Base(cost_model, aTbKey), _bTc(bTc), _aTc(aTc)
  {
  }

  virtual Eigen::VectorXd evaluateError(const SE3& aTb,
                                        boost::optional<Eigen::MatrixXd&> H = boost::none) const override
  {
    Eigen::Matrix<double, 6, 6> atcp_H_atb, err_H_atcp;
    const SE3 aTc_pred{ aTb.compose(_bTc, atcp_H_atb) };
    const Eigen::VectorXd error{ aTc_pred.logmap(_aTc, err_H_atcp) };

    if (H)
    {
      *H = err_H_atcp * atcp_H_atb;
    }

    return error;
  }

  const SE3 _bTc;
  const SE3 _aTc;
};

int main(int argc, char* argv[])
{
  SE3 c0_T_m121(0.0143374, 0.0283302, 0.99887, -0.03476, 1.59419, 0.80549, 2.60237);
  SE3 c1_T_m121(-0.0522592, 0.998291, -0.014217, 0.0233899, 2.0509, -1.0883, 2.8191);

  SE3 c0_T_m127(-0.031593, 0.962461, -0.268896, -0.009358, 1.8759, 0.11088, 2.6178);
  SE3 c1_T_m127(-0.0398539, 0.258178, 0.964094, -0.04616, 1.739, -0.40321, 2.7195);

  SE3 c0_T_m80(-0.009324, 0.964575, 0.261143, -0.03729, 1.3252, -0.3129, 2.6665);
  SE3 c1_T_m80(0.642206, -0.16972, 0.717232, 0.212586, 2.3111, 0.032843, 2.7238);

  SE3 c0_T_m1(0.00458, -0.485441, 0.874268, -0.0173, 1.4126, -1.1568, 2.7344);
  SE3 c1_T_m1(-0.0595009, 0.865724, 0.497005, 0.007383, 2.2774, 0.87803, 2.6858);

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial_values;

  const gtsam::Key c0_T_c1{ gtsam::Symbol('T', 0) };
  graph.emplace_shared<compose_factor_t>(c0_T_c1, c1_T_m1, c0_T_m1);
  graph.emplace_shared<compose_factor_t>(c0_T_c1, c1_T_m127, c0_T_m127);
  graph.emplace_shared<compose_factor_t>(c0_T_c1, c1_T_m121, c0_T_m121);
  // graph.emplace_shared<compose_factor_t>(c0_T_c1, c1_T_m80, c0_T_m80);

  const SE3 T_initial{ c0_T_m1 * c1_T_m1.inverse() };
  initial_values.insert(c0_T_c1, T_initial);
  PRX_DBG_VARS(T_initial);

  gtsam::LevenbergMarquardtParams lm_params{ prx::fg::default_levenberg_marquardt_parameters() };
  lm_params.setMaxIterations(100);

  PRX_MSG("Starting optimizer");

  gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_values, lm_params);
  gtsam::Values result{ optimizer.optimize() };

  const SE3 T_result{ result.at<SE3>(c0_T_c1) };
  PRX_DBG_VARS(T_result);

  return 0;
}