#include <iostream>

#ifndef JSON
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

#include <nlohmann/json.hpp>

using SF = prx::fg::symbol_factory_t;
using Translation = Eigen::Vector3d;
using SE3ObsFactor = prx::fg::SE3_observation_factor_t;
using ScrewAxis = prx::fg::screw_axis_t;
using SE3 = prx::fg::se3_t;
using Integrator = prx::fg::lie_integration_factor_t<SE3, ScrewAxis>;
using ScrewSmothing = prx::fg::screw_smoothing_factor_t;

gtsam::Key rod_symbol(const int rod, const int t)
{
  return SF::create_hashed_symbol("X^{", rod, "}_{", t, "}");
}

gtsam::Key rodvel_symbol(const int rod, const int t)
{
  return SF::create_hashed_symbol("\\dot{X}^{", rod, "}_{", t, "}");
}

int main(int argc, char* argv[])
{
  std::ifstream f(prx::lib_path + "/data/tensegrity/data.json");
  nlohmann::json json{ nlohmann::json::parse(f) };

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial_values;

  int t{ 0 };
  const Translation offset(0, 0, 3.25 / 2.0);
  gtsam::noiseModel::Base::shared_ptr z_noise{ gtsam::noiseModel::Isotropic::Sigma(3, 10) };

  std::ofstream ofs_zs(prx::out_path + "/tensegrity_observations.txt");
  ofs_zs << "# x1 y1 z1 x2 y2 z3\n";

  for (auto& element : json)
  {
    // std::cout << element["time"] << '\n';
    const Translation rand0{ prx::uniform_random<Translation>(0.0, 0.10) };
    const Translation rand1{ prx::uniform_random<Translation>(0.0, 0.10) };

    Translation r01pt1{ element["r01_end_pt1"].template get<std::vector<double>>().data() };
    Translation r01pt2{ element["r01_end_pt2"].template get<std::vector<double>>().data() };
    r01pt1 += rand0;
    r01pt2 += rand1;

    ofs_zs << r01pt1.transpose() << " ";
    ofs_zs << r01pt2.transpose() << "\n";

    graph.emplace_shared<SE3ObsFactor>(rod_symbol(1, t), -offset, r01pt1, z_noise);
    graph.emplace_shared<SE3ObsFactor>(rod_symbol(1, t), offset, r01pt2, z_noise);

    const Eigen::Quaterniond init_quat{ Eigen::Quaterniond::FromTwoVectors(Translation(0, 0, 1), r01pt2 - r01pt1) };
    const SE3 midpt01{ init_quat, (r01pt1 + r01pt2) / 2.0 };
    initial_values.insert(rod_symbol(1, t), midpt01);
    t++;
  }

  for (int i = 1; i < t - 1; ++i)
  {
    auto e0 = json[i - 1];
    auto e1 = json[i];
    const double t0{ e0["time"].template get<double>() };
    const double t1{ e1["time"].template get<double>() };
    // const double dt{ 1.0 };
    const double dt{ t1 - t0 };
    // std::cout << t0 << " " << t1 << '\n';
    gtsam::noiseModel::Base::shared_ptr xdot_noise{ gtsam::noiseModel::Isotropic::Sigma(6, 1) };

    // xt1 = xt0 + xdot * dt
    graph.emplace_shared<Integrator>(rod_symbol(1, i), rod_symbol(1, i - 1), rodvel_symbol(1, i - 1), xdot_noise, 1.0);

    const SE3 x0{ initial_values.at<SE3>(rod_symbol(1, i - 1)) };
    const SE3 x1{ initial_values.at<SE3>(rod_symbol(1, i)) };

    const SE3 x01{ x1 * x0.inverse() };
    const ScrewAxis screw_axis{ SE3::Logmap(x01) };
    // PRX_DBG_VARS(x0, x1, x01);
    // PRX_DBG_VARS(screw_axis);
    // PRX_DBG_VARS(Integrator::predict(x0, screw_axis, dt));
    initial_values.insert(rodvel_symbol(1, i - 1), screw_axis);
    // break;
  }
  const double Lwv{ 0.0010 };
  // const double Lwv{ 1.0 };
  gtsam::noiseModel::Base::shared_ptr Lwv_noise{ gtsam::noiseModel::Isotropic::Sigma(6, 0.01) };
  for (int i = 1; i < t - 2; ++i)
  {
    graph.emplace_shared<ScrewSmothing>(rodvel_symbol(1, i - 1), rodvel_symbol(1, i), Lwv, Lwv_noise);
    // graph.emplace_shared<gtsam::BetweenFactor<ScrewAxis>>(rodvel_symbol(1, i - 1), rodvel_symbol(1, i),
    //                                                       Lwv * Eigen::Vector<double, 6>::Ones(), Lwv_noise);
  }

  SF::symbols_to_file();
  gtsam::LevenbergMarquardtParams lm_params{ prx::fg::default_levenberg_marquardt_parameters() };
  lm_params.setUseFixedLambdaFactor(true);
  lm_params.setMaxIterations(100);

  gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_values, lm_params);

  gtsam::Values result{ optimizer.optimize() };

  // result.print("Result", SF::formatter);

  std::ofstream ofs_init(prx::lib_path + "/out/tensegrity_initial.txt");
  std::ofstream ofs_res(prx::lib_path + "/out/tensegrity_result.txt");

  ofs_init << "# Qw Qx Qy Qz x y z\n";
  ofs_res << "# Qw Qx Qy Qz x y z\n";
  for (int i = 0; i < t; ++i)
  {
    ofs_init << initial_values.at<SE3>(rod_symbol(1, i)) << "\n";
    ofs_res << result.at<SE3>(rod_symbol(1, i)) << "\n";
  }

  prx::fg::values_by_type_to_file<ScrewAxis>(initial_values, prx::out_path + "/tensegrity_initial_screw.txt");
  prx::fg::values_by_type_to_file<ScrewAxis>(result, prx::out_path + "/tensegrity_result_screw.txt");

  prx::fg::values_by_type_to_file<SE3>(initial_values, prx::out_path + "/tensegrity_initial_se3.txt");
  prx::fg::values_by_type_to_file<SE3>(result, prx::out_path + "/tensegrity_result_se3.txt");
  // initial_values.print("NonlinearFactorGraph", SF::formatter);
  // graph.printErrors(result, "NonlinearFactorGraph: ", SF::formatter);

  return 0;
}
#else
int main(int argc, char* argv[])
{
  std::cout << "JSON not built" << std::endl;
  return -1;
}
#endif