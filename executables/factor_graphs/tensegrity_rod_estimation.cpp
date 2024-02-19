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

#include "prx/factor_graphs/factors/SE3.hpp"
#include "prx/factor_graphs/factors/screw_axis.hpp"
#include "prx/factor_graphs/factors/preintegration.hpp"
#include "prx/factor_graphs/factors/position_velocity_factor.hpp"
#include "prx/factor_graphs/factors/smooth_factor.hpp"
#include "prx/factor_graphs/factors/euclidean_distance_factor.hpp"
#include "prx/factor_graphs/utilities/symbols_factory.hpp"
#include "prx/factor_graphs/utilities/default_parameters.hpp"
#include "prx/factor_graphs/utilities/common_functions.hpp"

#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/linear/JacobianFactor.h>

using prx::utilities::convert_to;
using prx::utilities::csv_reader_t;
using SF = prx::fg::symbol_factory_t;
using PositionVelFactor = prx::fg::position_velocity_factor_t;
using SmoothingFactor = prx::fg::smooth_factor_t<3>;
using DistanceFactor = prx::fg::euclidean_distance_factor_t<3>;

auto k_x = [](const std::size_t& ti, const std::size_t& i) {
  return SF::create_hashed_symbol("x^{", i, "}_{", ti, "}");
};
auto k_v = [](const std::size_t& ti, const std::size_t& i) {
  return SF::create_hashed_symbol("v^{", i, "}_{", ti, "}");
};

Eigen::Vector3d get_endcap_position(const csv_reader_t::Line<std::string>& line, const std::size_t& x_idx)
{
  const double x{ convert_to<double>(line[x_idx]) };
  const double y{ convert_to<double>(line[x_idx + 1]) };
  const double z{ convert_to<double>(line[x_idx + 2]) };
  return Eigen::Vector3d(x, y, z);
}
int main(int argc, char* argv[])
{
  prx::param_loader params("executables/factor_graphs/tensegrity_rod_estimation.yaml", argc, argv);

  params.print();
  prx::utilities::csv_reader_t reader(params["input/file"].as<>(), ' ');
  const std::size_t t_idx{ params["input/t_idx"].as<std::size_t>() };
  const std::size_t x0_idx{ params["input/x0_idx"].as<std::size_t>() };
  const std::size_t x1_idx{ params["input/x1_idx"].as<std::size_t>() };

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values0;
  gtsam::Values values1;

  Eigen::Vector3d posi_tm(Eigen::Vector3d::Zero());
  Eigen::Vector3d posj_tm(Eigen::Vector3d::Zero());
  bool first{ true };
  const std::size_t i{ 0 }, j{ 1 };
  std::size_t tn{ 0 };
  double t_prev{ 0.0 };
  const double lambda{ params["lambda"].as<double>() };
  const double rod_length{ params["rod_length"].as<double>() };
  auto noise_model = gtsam::noiseModel::Isotropic::Sigma(3, 1e-1);
  auto smoothing_nm = gtsam::noiseModel::Isotropic::Sigma(3, 1e0);
  auto prior_noise = gtsam::noiseModel::Isotropic::Sigma(3, 1e0);
  auto distance_nm = gtsam::noiseModel::Isotropic::Sigma(1, 1e0);
  while (reader.has_next_line())
  {
    auto line = reader.next_line();
    if (line.size() == 0)
      continue;
    // PRX_DEBUG_VAR_1(line[0]);
    const double t_curr{ convert_to<double>(line[t_idx]) };

    if (t_curr == t_prev)
      continue;
    const std::size_t tm{ tn - 1 };

    const Eigen::Vector3d posi_tn{ get_endcap_position(line, x0_idx) };
    const Eigen::Vector3d posj_tn{ get_endcap_position(line, x1_idx) };
    values0.insert(k_x(tn, i), posi_tn);
    values1.insert(k_x(tn, j), posj_tn);
    // graph.addPrior(k_x(j), pos_j, prior_noise);
    if (not first)
    {
      const Eigen::Vector3d vi_tm{ (posi_tn - posi_tm) / (t_curr - t_prev) };  // initial estimation
      const Eigen::Vector3d vj_tm{ (posj_tn - posj_tm) / (t_curr - t_prev) };  // initial estimation
      values0.insert(k_v(tm, i), vi_tm);
      values1.insert(k_v(tm, j), vj_tm);
      graph.emplace_shared<PositionVelFactor>(k_x(tm, i), k_x(tn, i), k_v(tm, i), t_prev, t_curr, noise_model, "0");
      graph.emplace_shared<PositionVelFactor>(k_x(tm, j), k_x(tn, j), k_v(tm, j), t_prev, t_curr, noise_model, "1");
      graph.emplace_shared<DistanceFactor>(k_x(tm, i), k_x(tm, j), rod_length, distance_nm);
    }
    posi_tm = posi_tn;
    posj_tm = posj_tn;
    first = false;
    tn++;
    t_prev = t_curr;
  }
  for (int tm = 0; tm < tn - 2; ++tm)
  {
    graph.emplace_shared<SmoothingFactor>(k_v(tm, i), k_v(tm + 1, i), lambda, smoothing_nm);
    graph.emplace_shared<SmoothingFactor>(k_v(tm, j), k_v(tm + 1, j), lambda, smoothing_nm);
  }
  SF::symbols_to_file();

  gtsam::LevenbergMarquardtParams lm_params{ prx::fg::default_levenberg_marquardt_parameters() };
  lm_params.verbosityLMTranslator(gtsam::LevenbergMarquardtParams::SILENT);
  lm_params.setMaxIterations(50);
  // lm_params.setMaxIterations(10000);
  lm_params.setRelativeErrorTol(1e-8);
  lm_params.setAbsoluteErrorTol(1e-8);
  lm_params.setlambdaUpperBound(1e64);
  lm_params.print("lm_params");

  gtsam::Values values{ values0 };
  values.insert(values1);
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
  gtsam::Values results = prx::fg::optimize_and_log(optimizer, lm_params);

  // prx::fg::values_to_file<Eigen::Vector3d>(results, "x_\\{\\d+\\}", params["out/positions_file"].as<>(),
  // "\\D|\\{|\\}"); prx::fg::values_to_file<Eigen::Vector3d>(results, "v_\\{\\d+\\}",
  // params["out/velocities_file"].as<>(),
  //                                          "\\D|\\{|\\}");

  const std::string out_filename0{ params["out/file0"].as<>() };
  const std::string out_filename1{ params["out/file1"].as<>() };
  std::ofstream ofs0(out_filename0, std::ofstream::trunc);
  std::ofstream ofs1(out_filename1, std::ofstream::trunc);

  for (auto factor : graph)
  {
    auto pv_factor = boost::dynamic_pointer_cast<PositionVelFactor>(factor);
    if (pv_factor)
    {
      pv_factor->eval_to_stream(results, ofs0);
    }
  }
  ofs0.close();
  ofs1.close();
  PRX_DEBUG_VAR_1(out_filename0);
  PRX_DEBUG_VAR_1(out_filename1);
  return 0;
}
