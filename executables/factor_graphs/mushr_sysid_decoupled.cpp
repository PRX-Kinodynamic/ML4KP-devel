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
using SF = prx::fg::symbol_factory_t;

using prx::fg::mushr_ub_u_xdot_param_t;
using prx::fg::mushr_ub_u_xdot_t;
using prx::fg::mushr_x_observation_t;
using prx::utilities::convert_to;
using namespace prx::fg::mushrTypes;

void add_observations(gtsam::NonlinearFactorGraph& graph, gtsam::Values& values, const std::string& filename,
                      const prx::trajectory_t& traj, const prx::plan_t& plan)
{
  prx::fg::mushrConfig config;
  config.cm_x_xdot = gtsam::noiseModel::Isotropic::Sigma(3, 1e-0);
  config.cm_x_xdot_ub = gtsam::noiseModel::Isotropic::Sigma(3, 1e-0);
  config.cm_ub_u = gtsam::noiseModel::Isotropic::Sigma(2, 1e-0);
  config.cm_x_z = gtsam::noiseModel::Diagonal::Sigmas(Eigen::Vector3d(1e0, 1e0, 1));
  config.length = 0.2965;

  ParamsDeltaVel params_dv{ ParamsDeltaVel(0.01) };
  values.insert(k_Ps("dv"), params_dv);
  const double duration{ plan.duration() };
  prx::utilities::csv_reader_t reader(filename, ' ');

  std::vector<double> ts{};
  std::vector<Eigen::Vector3d> xs{};

  PRX_DEBUG_VAR_1(duration);
  while (reader.has_next_line())
  {
    auto line = reader.next_line();
    if (line.size() > 0)
    {
      const double t{ convert_to<double>(line[2]) };
      const double x{ convert_to<double>(line[3]) };
      const double y{ convert_to<double>(line[4]) };
      const double qw{ convert_to<double>(line[6]) };
      const double qx{ convert_to<double>(line[7]) };
      const double qy{ convert_to<double>(line[8]) };
      const double qz{ convert_to<double>(line[9]) };
      const double theta{ prx::yaw(Eigen::Quaterniond(qw, qx, qy, qz)) };

      ts.emplace_back(t);
      xs.emplace_back(x, y, theta);
    }
  }
  double t_prev{ 0.0 };
  double t_now{ 0.0 };
  double t_accum{ 0.0 };
  double dt{ 0.0 };
  Eigen::Vector3d zt{};
  Eigen::Vector3d zt1{};
  std::size_t ti{ 0 };
  prx::space_point_t x_aux{};
  Eigen::Vector3d xt{};
  Eigen::Vector3d xt_next{};
  Eigen::Vector3d xdt{};
  Eigen::Vector2d ut{};
  Eigen::Vector2d ubar{};
  bool first{ true };
  auto nm_u_prior = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);

  xt = Vec(traj.at(t_accum)).head(3);
  xdt = Vec(traj.at(t_accum)).tail(3);
  zt1 = xs[0];
  values.insert(k_X(0), xt);
  values.insert(k_Xd(0), xdt);
  graph.emplace_shared<mushr_x_observation_t>(k_X(0), zt1, config.cm_x_z);

  for (int i = 0; i < ts.size() - 1; ++i)
  {
    t_now = ts[i];
    zt = xs[i];
    zt1 = xs[i + 1];
    dt = ts[i + 1] - ts[i];

    for (int j = 0; j < 3; ++j)
    {
      add_mushr_factor_graph_step(ti + j, dt / 3.0, graph, config);
    }

    const double t01{ t_accum / duration };              // current t \in [0,1]
    const double t01_next{ (t_accum + dt) / duration };  // current t \in [0,1]
    x_aux = traj.at(t01);
    // xt = Vec(x_aux).head(3);
    ut = Vec(plan.at(t_accum));
    ubar = mushr_ub_u_xdot_param_t::predict(ut, xdt, params_dv, config.length);
    xt = Vec(traj.at(t01_next)).head(3);
    xdt = Vec(traj.at(t01_next)).tail(3);
    // ubar = mushr_ub_u_xdot_t::predict(ut, xdt, config.length);

    graph.emplace_shared<mushr_x_observation_t>(k_X(ti + 3), zt1, config.cm_x_z);

    for (int j = 0; j < 3; ++j)
    {
      PRX_DEBUG_VAR_2(ti, k_Xd(ti + 1));
      values.insert(k_X(ti + 1), xt);
      values.insert(k_Xd(ti + 1), xdt);
      values.insert(k_U(ti), ut);
      values.insert(k_Ub(ti), ubar);
      graph.addPrior(k_U(ti), ut, nm_u_prior);
      ti++;
    }
    SF::symbols_to_file();
    // graph.print("Graph", prx::fg::symbol_factory_t::formatter);
    // PRX_DEBUG_VAR_1(graph.linearize(values)->jacobian().first);
    // PRX_DEBUG_VAR_1(graph.linearize(values)->augmentedHessian());
    // graph.orderingCOLAMD().print("Ordering: ", prx::fg::symbol_factory_t::formatter);

    t_accum += dt;
  }
  // const Eigen::Vector3d last_x{ Vec(traj.at(1.0)).head(3) };
  // values.insert(k_X(ti), last_x);
}

int main(int argc, char* argv[])
{
  const std::string params_file{ "executables/factor_graphs/mushr_sysid.yaml" };
  prx::param_loader params{ params_file, argc, argv };
  prx::simulation_step = 0.01;

  const std::string plant_name{ "mushrFG" };
  const std::string plant_path{ "mushrFG" };
  auto plant = prx::system_factory_t::create_system(plant_name, plant_path);

  prx::world_model_t world_model({ plant }, {});
  const std::string context_name{ "mushrFG" };
  world_model.create_context(context_name, { plant_name }, {});
  prx::world_model_context context{ world_model.get_context(context_name) };
  std::shared_ptr<prx::system_group_t> sys_group{ context.first };

  prx::space_t* ss{ sys_group->get_state_space() };
  prx::space_t* cs{ sys_group->get_control_space() };
  prx::space_t* ps{ sys_group->get_parameter_space() };

  ps->copy_from({ 0.5 });
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;

  prx::plan_t plan{ cs };
  prx::trajectory_t traj{ ss };
  prx::space_point_t start_state{ ss->make_point() };

  plan.from_file(params["plan"].as<std::string>());
  Vec(start_state) = Eigen::Vector<double, 6>::Zero();
  sys_group->propagate(start_state, plan, traj);
  traj.to_file(prx::out_path + "mushr/orig_traj.txt");

  PRX_DEBUG_VAR_1(plan);
  add_observations(graph, values, params["observations"].as<>(), traj, plan);

  gtsam::LevenbergMarquardtParams lm_params{ prx::fg::default_levenberg_marquardt_parameters() };
  lm_params.verbosityLMTranslator(gtsam::LevenbergMarquardtParams::SILENT);
  lm_params.setMaxIterations(50);
  // lm_params.setMaxIterations(10000);
  lm_params.setRelativeErrorTol(1e-8);
  lm_params.setAbsoluteErrorTol(1e-8);
  lm_params.setlambdaUpperBound(1e64);
  lm_params.print("lm_params");

  SF::symbols_to_file();

  // graph.printErrors(values, "Errors: ", prx::fg::symbol_factory_t::formatter,
  //                   [](const gtsam::Factor*, double error, std::size_t) { return error > 0.001; });

  gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
  gtsam::Values results = prx::fg::optimize_and_log(optimizer, lm_params);
  // gtsam::Values results = values;
  const std::string out_filename{ params["out/file"].as<>() };
  std::ofstream ofs(out_filename, std::ofstream::trunc);

  for (auto factor : graph)
  {
    auto factor0 = boost::dynamic_pointer_cast<prx::fg::mushr_x_xdot_t>(factor);
    auto factor1 = boost::dynamic_pointer_cast<prx::fg::mushr_x_xdot_ub_t>(factor);
    auto factor2 = boost::dynamic_pointer_cast<prx::fg::mushr_ub_u_xdot_t>(factor);
    auto factor3 = boost::dynamic_pointer_cast<prx::fg::mushr_x_observation_t>(factor);
    if (factor0)  // mushr_ub_u_xdot_t
    {
      ofs << "mushr_x_xdot_t ";
      factor0->eval_to_stream(results, ofs);
    }
    // if (factor1)
    // {
    //   ofs << "mushr_x_xdot_ub_t ";
    //   factor1->eval_to_stream(results, ofs);
    // }
    // if (factor2)
    // {
    //   ofs << "mushr_ub_u_xdot_t ";
    //   factor2->eval_to_stream(results, ofs);
    // }
    // if (factor3)
    // {
    //   ofs << "mushr_x_observation_t ";
    //   factor3->eval_to_stream(results, ofs);
    // }
  }

  const prx::fg::mushrTypes::ParamsDeltaVel params_out{ results.at<prx::fg::mushrTypes::ParamsDeltaVel>(k_Ps("dv")) };
  PRX_DEBUG_VAR_1(params_out);
  ps->copy_from(params_out);
  prx::trajectory_t res_traj{ ss };
  sys_group->propagate(start_state, plan, res_traj);
  res_traj.to_file(prx::out_path + "mushr/res_traj.txt");

  ofs.close();
  PRX_DEBUG_VAR_1(out_filename);
  return 0;
}