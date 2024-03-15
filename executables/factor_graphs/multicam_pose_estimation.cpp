#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>

#include <boost/algorithm/string/join.hpp>
#include <boost/optional.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/ISAM2.h>

#include "prx/simulation/plants/plants.hpp"
#include "prx/simulation/playback/trajectory.hpp"
#include "prx/simulation/system_factory.hpp"
#include "prx/simulation/system_group.hpp"
#include "prx/planning/world_model.hpp"

#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/csv_reader.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/visualization/three_js_group.hpp"

#include <gtsam/inference/Key.h>
#include <gtsam/linear/NoiseModel.h>
// #include "prx/factor_graphs/defs.hpp"
#include "prx/factor_graphs/utilities/default_parameters.hpp"
#include "prx/factor_graphs/factors/SE3.hpp"
#include "prx/factor_graphs/factors/screw_axis.hpp"
#include "prx/factor_graphs/factors/preintegration.hpp"
#include "prx/factor_graphs/factors/position_velocity_factor.hpp"
#include "prx/factor_graphs/factors/smooth_factor.hpp"
#include "prx/factor_graphs/factors/mushr_factors.hpp"
#include "prx/factor_graphs/utilities/symbols_factory.hpp"
#include "prx/factor_graphs/utilities/common_functions.hpp"
#include "prx/factor_graphs/factors/transform_factor.hpp"
// #include "prx/factor_graphs/utilities/utilities_functions.hpp"
// #include "prx/factor_graphs/factors/factors.hpp"

// #include <gtsam/nonlinear/Marginals.h>
// #include <gtsam/linear/JacobianFactor.h>

using prx::utilities::convert_to;
using SF = prx::fg::symbol_factory_t;

auto k_C = [](const std::string& ci) { return SF::create_hashed_symbol("C_{", ci, "}"); };
auto k_M = [](const std::string& ci, const std::string& mi) {
  return SF::create_hashed_symbol("^", ci, "M_{", mi, "}");
};
auto k_T = [](const std::string& ci, const std::string& mi) {
  return SF::create_hashed_symbol("^{", ci, "}_T_{", mi, "}");
};

using CameraData = std::tuple<int, int, double, Eigen::Vector3d, Eigen::Quaterniond>;
std::vector<CameraData> get_data()
{
  std::vector<CameraData> cam{};
  cam.push_back(std::make_tuple(0, 1, 1708991104.508039356, Eigen::Vector3d(-1.72686, -1.11315, 2.66006),
                                Eigen::Quaterniond(0.0294901, -0.0168647, -0.042476, 0.99852)));
  cam.push_back(std::make_tuple(0, 51, 1708991104.541137909, Eigen::Vector3d(0.86813, -1.19979, 2.73257),
                                Eigen::Quaterniond(0.0234487, 0.0147466, 0.735343, -0.677128)));
  cam.push_back(std::make_tuple(0, 62, 1708991104.508362383, Eigen::Vector3d(1.06688, 0.89109, 2.65199),
                                Eigen::Quaterniond(-0.00517006, -0.0392946, 0.0361386, 0.998561)));
  cam.push_back(std::make_tuple(0, 80, 1708991104.508505293, Eigen::Vector3d(-1.55779, 1.00962, 2.48693),
                                Eigen::Quaterniond(0.0185345, -0.0352959, 0.023271, 0.998934)));
  cam.push_back(std::make_tuple(0, 120, 1708991104.508639807, Eigen::Vector3d(2.03171, -1.03251, 2.65534),
                                Eigen::Quaterniond(0.0251542, 0.00464513, 0.911742, -0.409966)));
  cam.push_back(std::make_tuple(0, 121, 1708991104.508772825, Eigen::Vector3d(1.92796, 0.765732, 2.57728),
                                Eigen::Quaterniond(0.0430708, -0.00286288, 0.99752, -0.055589)));

  cam.push_back(std::make_tuple(1, 120, 1708991104.512784311, Eigen::Vector3d(1.63243, 0.738261, 2.5842),
                                Eigen::Quaterniond(-0.0114038, -0.0365983, 0.917892, 0.394973)));
  cam.push_back(std::make_tuple(1, 121, 1708991104.513107837, Eigen::Vector3d(1.9142, -1.04764, 2.71031),
                                Eigen::Quaterniond(-0.0201428, -0.020555, 0.715212, 0.698315)));
  cam.push_back(std::make_tuple(1, 122, 1708991104.513422746, Eigen::Vector3d(-1.68362, -1.16792, 2.64205),
                                Eigen::Quaterniond(-0.0289132, -0.00219618, -0.361284, 0.932005)));
  cam.push_back(std::make_tuple(1, 123, 1708991104.513724582, Eigen::Vector3d(-1.75368, 0.768699, 2.52382),
                                Eigen::Quaterniond(0.0110982, -0.035389, 0.996025, 0.0809832)));
  cam.push_back(std::make_tuple(1, 124, 1708991104.514061195, Eigen::Vector3d(1.35413, -0.181948, 2.62351),
                                Eigen::Quaterniond(0.0176672, -0.0310529, 0.961823, -0.271331)));
  return cam;
}

int main(int argc, char* argv[])
{
  const std::string params_file{ "executables/factor_graphs/multicam_pose_estimation.yaml" };
  prx::param_loader params{ params_file, argc, argv };

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  auto mushr_prop_nm = gtsam::noiseModel::Isotropic::Sigma(4, 1e-1);
  auto z_prior_nm = gtsam::noiseModel::Diagonal::Sigmas(Eigen::Vector4d(1e0, 1e0, 1e0, 1e0));
  auto u_prior_nm = gtsam::noiseModel::Diagonal::Sigmas(Eigen::Vector2d(1e-1, 1e-1));

  auto tf_nm = gtsam::noiseModel::Isotropic::Sigma(6, 1e0);
  std::vector<CameraData> cam_data = get_data();
  using TransformFactor = prx::fg::transform_factor_t;

  std::ofstream ofs_dbg(prx::out_path + "/dbg_multicam.txt", std::ofstream::trunc);

  const gtsam::Pose3 C0Tw{ gtsam::Rot3(Eigen::Quaterniond(0.0430708, -0.00286288, 0.99752, -0.055589)),
                           Eigen::Vector3d(1.92796, 0.765732, 2.57728) };
  const gtsam::Pose3 C1Tw{ gtsam::Rot3(Eigen::Quaterniond(-0.0201428, -0.020555, 0.715212, 0.698315)),
                           Eigen::Vector3d(1.9142, -1.04764, 2.71031) };
  // ^WT_{Mx} = ^WT_{C0} * ^{C0}T_{Mx}
  for (auto cd : cam_data)
  {
    const std::string Ci{ "C" + convert_to<std::string>(std::get<0>(cd)) };
    const std::string Mi{ "M" + convert_to<std::string>(std::get<1>(cd)) };
    const double ti{ std::get<2>(cd) };
    const Eigen::Vector3d pi{ std::get<3>(cd) };
    const Eigen::Quaterniond qi{ std::get<4>(cd) };

    const gtsam::Pose3 CTMi(gtsam::Rot3(qi), pi);
    const gtsam::Pose3 WTC{ Ci == "C0" ? C0Tw.inverse() : C1Tw.inverse() };
    ofs_dbg << Mi << Ci << " ";
    ofs_dbg << TransformFactor::to_string((WTC * CTMi).translation().transpose()) << " ";
    ofs_dbg << TransformFactor::to_string((WTC * CTMi).rotation().toQuaternion()) << " ";
    ofs_dbg << "\n";
    // PRX_DEBUG_VAR_2(Ci, Mi);
    values.insert(k_M(Ci, Mi), WTC * CTMi);
    // PRX_DEBUG_VAR_1(WTC * CTMi);
    values.insert(k_T(Ci, Mi), CTMi);
    graph.emplace_shared<TransformFactor>(k_C(Ci), k_M(Ci, Mi), k_T(Ci, Mi), tf_nm);
  }
  ofs_dbg.close();

  // C0: -2, 1.0, 2.4 Q: 1,0,0,0
  values.insert(k_C("C0"), gtsam::Pose3(gtsam::Rot3(Eigen::Quaterniond(0, 1, 0, 0)), Eigen::Vector3d(-2, 1.0, 2.4)));
  // C1: 1.70, 1.0, 2.8 Q: 0,1,0,0
  values.insert(k_C("C1"), gtsam::Pose3(gtsam::Rot3(Eigen::Quaterniond(0, 0, 1, 0)), Eigen::Vector3d(1.70, 1.0, 2.8)));

  // C0T_C1 3.735 -0.45 -.05 0 0 .998 .0457
  graph.emplace_shared<TransformFactor>(k_C("C0"), k_C("C1"), k_T("C1", "C0"), tf_nm);
  values.insert(k_T("C1", "C0"),
                gtsam::Pose3(gtsam::Rot3(Eigen::Quaterniond(0, 0, 1, 0)), Eigen::Vector3d(3.735, -0.45, -0.05)));

  gtsam::LevenbergMarquardtParams lm_params{ prx::fg::default_levenberg_marquardt_parameters() };
  lm_params.verbosityLMTranslator(gtsam::LevenbergMarquardtParams::SILENT);
  lm_params.setMaxIterations(50);
  // lm_params.setMaxIterations(10000);
  lm_params.setRelativeErrorTol(1e-8);
  lm_params.setAbsoluteErrorTol(1e-8);
  lm_params.setlambdaUpperBound(1e64);
  lm_params.print("lm_params");

  SF::symbols_to_file();

  gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
  gtsam::Values results = prx::fg::optimize_and_log(optimizer, lm_params);

  const std::string out_filename{ params["out/file"].as<>() };
  std::ofstream ofs(out_filename, std::ofstream::trunc);

  for (auto factor : graph)
  {
    auto pv_factor = boost::dynamic_pointer_cast<TransformFactor>(factor);
    if (pv_factor)
    {
      pv_factor->eval_to_stream(results, ofs);
    }
  }
  ofs.close();
  PRX_DEBUG_VAR_1(out_filename);
  return 0;
}