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
// #include "prx/factor_graphs/utilities/utilities_functions.hpp"
// #include "prx/factor_graphs/factors/factors.hpp"

// #include <gtsam/nonlinear/Marginals.h>
// #include <gtsam/linear/JacobianFactor.h>

using prx::utilities::convert_to;
using SF = prx::fg::symbol_factory_t;

auto k_X = [](const std::size_t& ti) { return SF::create_hashed_symbol("x_{", ti, "}"); };
auto k_U = [](const std::size_t& ti) { return SF::create_hashed_symbol("u_{", ti, "}"); };
auto k_Z = [](const std::size_t& ti) { return SF::create_hashed_symbol("z_{", ti, "}"); };
auto k_P = [](const std::size_t& ti) { return SF::create_hashed_symbol("theta_{", ti, "}"); };

int main(int argc, char* argv[])
{
  const std::string params_file{ "executables/factor_graphs/mushr_sysid.yaml" };
  prx::param_loader params{ params_file, argc, argv };
  prx::simulation_step = params["simulation_step"].as<double>();

  const std::string plant_name{ "mushr" };
  const std::string plant_path{ "mushr" };
  auto plant = prx::system_factory_t::create_system(plant_name, plant_path);

  prx::world_model_t world_model({ plant }, {});
  const std::string context_name{ "mushr" };
  world_model.create_context(context_name, { plant_name }, {});
  prx::world_model_context context{ world_model.get_context(context_name) };
  std::shared_ptr<prx::system_group_t> sys_group{ context.first };

  prx::space_t* ss{ sys_group->get_state_space() };
  prx::space_t* cs{ sys_group->get_control_space() };
  prx::space_t* ps{ sys_group->get_parameter_space() };

  ps->copy_from(params["/plant/parameter_space/values"].as<std::vector<double>>());

  const std::string filename{ params["tf_data"].as<>() };
  prx::utilities::csv_reader_t reader(filename, ' ');

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  auto mushr_prop_nm = gtsam::noiseModel::Isotropic::Sigma(4, 1e-1);
  auto z_prior_nm = gtsam::noiseModel::Diagonal::Sigmas(Eigen::Vector4d(1e0, 1e0, 1e0, 1e0));
  auto u_prior_nm = gtsam::noiseModel::Diagonal::Sigmas(Eigen::Vector2d(1e-1, 1e-1));

  std::size_t idx{ 0 };
  bool first{ true };
  double t_curr{ 0.0 };
  double t_prev{ 0.0 };
  Eigen::Vector4d state_prev{ Eigen::Vector4d::Zero() };

  const Eigen::Vector2d ctrl(-0.75, 0.5);
  const Eigen::Vector2d ctrl0(0, 0);
  const double ctrl_duration{ 20 };
  const double start_time{ 1708018322.353523000 };
  values.insert(k_P(0), Eigen::Vector<double, 6>(0.08, -0.08, 0.75, 0.0, -1.0, 1.0));

  while (reader.has_next_line())
  {
    auto line = reader.next_line();
    if (line.size() > 0)
    {
      t_curr = convert_to<double>(line[2]);
      const double x{ convert_to<double>(line[3]) };
      const double y{ convert_to<double>(line[4]) };
      const double qw{ convert_to<double>(line[6]) };
      const double qx{ convert_to<double>(line[7]) };
      const double qy{ convert_to<double>(line[8]) };
      const double qz{ convert_to<double>(line[9]) };
      const double theta{ prx::yaw(Eigen::Quaterniond(qw, qx, qy, qz)) };

      if (first)
      {
        state_prev = Eigen::Vector4d(x, y, theta, 0);
        values.insert(k_X(idx), state_prev);
        graph.addPrior(k_X(idx), state_prev, z_prior_nm);
      }
      else
      {
        const double dt{ t_curr - t_prev };
        graph.emplace_shared<prx::fg::mushr_factor_t>(k_X(idx - 1), k_X(idx), k_U(idx - 1), k_P(0), dt, mushr_prop_nm);
        const double vel{ std::sqrt(std::pow(x - state_prev[0], 2) + std::pow(y - state_prev[1], 2)) };
        state_prev = Eigen::Vector4d(x, y, theta, vel);
        values.insert(k_X(idx), state_prev);
        graph.addPrior(k_X(idx), state_prev, z_prior_nm);

        if (start_time < t_curr || t_curr < start_time + ctrl_duration)
        {
          values.insert(k_U(idx - 1), ctrl0);
          graph.addPrior(k_U(idx - 1), ctrl0, u_prior_nm);
        }
        else
        {
          values.insert(k_U(idx - 1), ctrl);
          graph.addPrior(k_U(idx - 1), ctrl, u_prior_nm);
        }
      }
      first = false;
      t_prev = t_curr;
      idx++;
    }
  }

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
    auto pv_factor = boost::dynamic_pointer_cast<prx::fg::mushr_factor_t>(factor);
    if (pv_factor)
    {
      pv_factor->eval_to_stream(results, ofs);
    }
  }
  ofs.close();
  PRX_DEBUG_VAR_1(out_filename);
  return 0;
}