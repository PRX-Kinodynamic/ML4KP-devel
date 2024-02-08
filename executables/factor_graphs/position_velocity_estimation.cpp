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
#include "prx/factor_graphs/utilities/symbols_factory.hpp"
#include "prx/factor_graphs/utilities/default_parameters.hpp"
#include "prx/factor_graphs/utilities/common_functions.hpp"

#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/linear/JacobianFactor.h>

using prx::utilities::convert_to;
using SF = prx::fg::symbol_factory_t;

auto k_x = [](const std::size_t& ti) { return SF::create_hashed_symbol("x_{", ti, "}"); };
auto k_v = [](const std::size_t& ti) { return SF::create_hashed_symbol("v_{", ti, "}"); };

int main(int argc, char* argv[])
{
  prx::param_loader params("executables/factor_graphs/position_velocity_estimation.yaml");

  prx::utilities::csv_reader_t reader(params["input/file"].as<>(), ' ');
  const std::size_t t_idx{ params["input/t_idx"].as<std::size_t>() };
  const std::size_t x_idx{ params["input/x_idx"].as<std::size_t>() };

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;

  Eigen::Vector3d pos_i(Eigen::Vector3d::Zero());
  bool first{ true };
  std::size_t i{ 0 }, j{ 1 };
  double ti{ 0.0 };
  const double lambda{ params["lambda"].as<double>() };
  auto noise_model = gtsam::noiseModel::Isotropic::Sigma(3, 1e-1);
  auto smoothing_nm = gtsam::noiseModel::Isotropic::Sigma(3, 1e0);
  auto prior_noise = gtsam::noiseModel::Isotropic::Sigma(3, 1e-20);
  while (reader.has_next_line())
  {
    auto line = reader.next_line();
    if (line.size() == 0)
      continue;
    // PRX_DEBUG_VAR_1(line[0]);
    const double tj{ convert_to<double>(line[t_idx]) };
    // PRX_DEBUG_VAR_1(t_i);
    const double xi{ convert_to<double>(line[x_idx]) };
    const double yi{ convert_to<double>(line[x_idx + 1]) };
    const double zi{ convert_to<double>(line[x_idx + 2]) };
    if (tj == ti)
      continue;
    const Eigen::Vector3d pos_j(xi, yi, zi);
    values.insert(k_x(j), pos_j);
    graph.addPrior(k_x(j), pos_j, prior_noise);
    if (not first)
    {
      const Eigen::Vector3d vi{ (pos_j - pos_i) / (tj - ti) };  // initial estimation
      values.insert(k_v(i), vi);
      graph.emplace_shared<prx::fg::position_velocity_factor_t>(k_x(i), k_x(j), k_v(i), ti, tj, noise_model);
      graph.emplace_shared<prx::fg::smooth_factor_t<3>>(k_v(i), k_v(j), lambda, smoothing_nm);
    }

    ti = tj;
    pos_i = pos_j;
    first = false;
    i++;
    j++;
  }
  graph.erase(graph.end() - 1);

  gtsam::LevenbergMarquardtParams lm_params{ prx::fg::default_levenberg_marquardt_parameters() };
  lm_params.verbosityLMTranslator(gtsam::LevenbergMarquardtParams::SILENT);
  lm_params.setMaxIterations(50);
  // lm_params.setMaxIterations(10000);
  lm_params.setRelativeErrorTol(1e-8);
  lm_params.setAbsoluteErrorTol(1e-8);
  lm_params.setlambdaUpperBound(1e64);
  lm_params.print("lm_params");

  gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
  gtsam::Values results = prx::fg::optimize_and_log(optimizer, lm_params);

  // prx::fg::values_to_file<Eigen::Vector3d>(results, "x_\\{\\d+\\}", params["out/positions_file"].as<>(),
  // "\\D|\\{|\\}"); prx::fg::values_to_file<Eigen::Vector3d>(results, "v_\\{\\d+\\}",
  // params["out/velocities_file"].as<>(),
  //                                          "\\D|\\{|\\}");

  std::ofstream ofs(params["out/file"].as<>(), std::ofstream::trunc);

  for (auto factor : graph)
  {
    auto pv_factor = boost::dynamic_pointer_cast<prx::fg::position_velocity_factor_t>(factor);
    if (pv_factor)
    {
      pv_factor->eval_to_stream(results, ofs);
    }
  }
  ofs.close();
  return 0;
}
