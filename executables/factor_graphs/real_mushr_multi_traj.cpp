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
#include "prx/factor_graphs/factors/positive_vector_factor.hpp"
using SF = prx::fg::symbol_factory_t;

using prx::fg::mushr_ub_u_xdot_param_t;
using prx::fg::mushr_ub_u_xdot_t;
using prx::fg::mushr_x_async_observation_t;
using prx::fg::mushr_x_observation_t;
using prx::utilities::convert_to;
using namespace prx::fg::mushrTypes;

using ObservedTrajectory = std::vector<std::pair<double, Eigen::Vector3d>>;

void read_observations(const std::string& filename, ObservedTrajectory& observations)
{
  PRX_DEBUG_VAR_1(filename);
  prx::utilities::csv_reader_t reader(filename, ' ');
  while (reader.has_next_line())
  {
    auto line = reader.next_line();
    if (line.size() > 0)
    {
      const std::string frame{ line[1] };
      if (frame != "robot_0")
        continue;
      const double t{ convert_to<double>(line[2]) };
      const double x{ convert_to<double>(line[3]) };
      const double y{ convert_to<double>(line[4]) };
      const double qw{ convert_to<double>(line[6]) };
      const double qx{ convert_to<double>(line[7]) };
      const double qy{ convert_to<double>(line[8]) };
      const double qz{ convert_to<double>(line[9]) };
      const double theta{ prx::yaw(Eigen::Quaterniond(qw, qx, qy, qz)) };

      // ts.emplace_back(t);
      observations.emplace_back(std::piecewise_construct, std::forward_as_tuple(t), std::forward_as_tuple(x, y, theta));
    }
  }
  PRX_DEBUG_VAR_1(observations.size());
}

double read_ros_plan(const std::string& filename, prx::plan_t& plan)
{
  plan.clear();
  prx::utilities::csv_reader_t reader(filename, ' ');
  std::vector<double> ts;
  std::vector<Eigen::Vector2d> observations;
  while (reader.has_next_line())
  {
    auto line = reader.next_line();
    if (line.size() > 0)
    {
      const double t{ convert_to<double>(line[0]) };
      const double u0{ convert_to<double>(line[1]) };
      const double u1{ convert_to<double>(line[2]) };
      ts.emplace_back(t);
      observations.emplace_back(u0, u1);
    }
  }
  double ti{ 0.0 };
  Eigen::Vector2d ut{};
  double tprev{ ts[0] };
  bool first{ true };
  for (auto tuple : prx::zip_iters(ts, observations))
  {
    if (not first)
    {
      std::tie(ti, ut) = prx::unzip(tuple);
      const double duration{ ti - tprev };
      plan.copy_onto_back(ut, duration);
      tprev = ti;
    }
    first = false;
  }
  return ts[0];
}

// Duration of observed trajectory is higher than plan (perception runs before plan publisher / controller and after...)
// Append zeros before and after assuming the robot was not moving before/after.
void increase_plan_to_match_trajectory(ObservedTrajectory& traj, const double plan_t0, prx::plan_t& plan,
                                       const std::size_t idx)
{
  const double traj_t0{ traj.front().first };
  const double traj_duration{ traj.back().first - traj_t0 };
  // const double plan_duration{ plan.duration() };
  // prx_assert(plan_duration < traj_duration, "plan duration is not less than traj duration!");
  while (plan.duration() >= traj_duration)
  {
    plan.pop_back();
  }

  while (traj[1].first < plan_t0)
  {
    traj.erase(traj.begin());
  }

  // Add controls at the end to ensure the predicted traj is long enough
  // const double end_diff{ traj_duration - plan_duration };
  // plan.copy_onto_front(plan[0].control, init_diff);
  // plan.copy_onto_back(plan.back().control, end_diff);
  // PRX_DEBUG_VAR_3(traj_duration, plan_duration, plan.duration());

  const double new_duration{ plan.duration() };
  std::ios_base::openmode mode{ idx == 0 ? std::ofstream::trunc : std::ofstream::app };
  std::ofstream ofs(prx::out_path + "mushr/input_observations.txt", mode);
  for (auto z : traj)
  {
    if (z.first - traj_t0 < new_duration)
      ofs << z.second.transpose() << "\n";
  }
  ofs << "\n";
  ofs.close();
}

void add_observations(gtsam::NonlinearFactorGraph& graph, gtsam::Values& values, const prx::trajectory_t& traj,
                      const prx::plan_t& plan, ObservedTrajectory& observations,
                      prx::fg::mushrTypes::ParamsUbarU& params_dv, std::size_t idx_offset)
{
  prx::fg::mushrConfig config;
  config.cm_x_xdot = gtsam::noiseModel::Isotropic::Sigma(3, 5e-6);
  config.cm_x_xdot_ub = gtsam::noiseModel::Isotropic::Sigma(3, 5e-2);
  config.cm_ub_u = gtsam::noiseModel::Isotropic::Sigma(prx::fg::mushrTypes::UbarDim, 5e-2);
  // config.cm_x_z = gtsam::noiseModel::Isotropic::Sigma(3, 1e-2);
  config.cm_x_z = gtsam::noiseModel::Diagonal::Sigmas(Eigen::Vector3d(1e0, 1e0, 1e0));
  config.length = 0.2965;

  // ParamsUbarU params_dv{ ParamsUbarU(0.1, 0.0) };
  const double duration{ plan.duration() };

  double t_prev{ 0.0 };
  double t_now{ 0.0 };
  double t_accum{ 0.0 };
  double dt{ 0.0 };
  prx::space_point_t x_aux{};
  Eigen::Vector3d xt{};
  Eigen::Vector3d xt_next{};
  Eigen::Vector3d xdt{};
  Eigen::Vector3d xdt_next{};
  Eigen::Vector2d ut{};
  prx::fg::mushrTypes::Ubar ubar{};
  bool first{ true };
  auto nm_u_prior = gtsam::noiseModel::Isotropic::Sigma(2, 1e-4);

  // Add the first step
  xt = Vec(traj.at(t_accum)).head(3);
  xdt = Vec(traj.at(t_accum)).tail(3);

  double ti{ 0.0 };
  std::size_t idx{ 0 };

  const double tobs_0{ observations[0].first };
  // const double tobs_T{ tobs_0 + duration };
  PRX_DEBUG_VAR_2(plan.duration(), traj.duration());
  const std::size_t traj_size{ traj.size() - 1 };
  const std::size_t max_idx{ std::min(traj_size, traj.index_at_time(duration)) };
  // PRX_DEBUG_VAR_3(tobs_0, tobs_T, max_idx);
  xt = Vec(traj[static_cast<unsigned>(idx)]).head(3);
  graph.addPrior(k_X(idx, idx_offset), xt, config.cm_x_xdot);

  for (; idx < max_idx; ++idx)
  {
    // config.cm_x_xdot = gtsam::noiseModel::Isotropic::Sigma(3, std::exp(ti));
    add_mushr_factor_graph_step(idx, prx::simulation_step, graph, config, idx_offset);

    xt = Vec(traj[static_cast<unsigned>(idx)]).head(3);
    xdt = Vec(traj[static_cast<unsigned>(idx)]).tail(3);
    ut = Vec(plan.at(ti));

    ubar = mushr_ub_u_xdot_param_t::predict(ut, xdt, params_dv, config.length);
    values.insert(k_X(idx, idx_offset), xt);
    values.insert(k_Xd(idx, idx_offset), xdt);
    values.insert(k_U(idx, idx_offset), ut);
    values.insert(k_Ub(idx, idx_offset), ubar);
    graph.addPrior(k_U(idx, idx_offset), ut, nm_u_prior);

    ti += prx::simulation_step;
    PRX_DEBUG_VAR_3(idx, max_idx, traj.size());
  }
  // xt = Vec(traj[static_cast<unsigned>(idx)]).head(3);
  // x_aux = traj.at(ti, false);
  PRX_DEBUG_VAR_1(idx);
  xt = Vec(traj[static_cast<unsigned>(idx)]).head(3);
  xdt = Vec(traj[static_cast<unsigned>(idx)]).tail(3);
  values.insert(k_X(idx, idx_offset), xt);
  values.insert(k_Xd(idx, idx_offset), xdt);
  SF::symbols_to_file();

  ti = 0;
  idx = 0;

  Eigen::Vector3d zt{};
  for (auto tuple : observations)
  {
    auto [ti, zt] = tuple;

    const double tobs{ ti - tobs_0 };
    PRX_DEBUG_VAR_3(tobs, ti, tobs_0);
    if (tobs < duration)
    {
      idx = traj.index_at_time(tobs);
      PRX_DEBUG_VAR_3(ti, tobs, duration);
      const double t0{ prx::simulation_step * idx };
      const double t1{ prx::simulation_step * (idx + 1) };
      if (idx < max_idx)
      {
        PRX_DEBUG_VAR_3(idx, t0, t1);
        PRX_DEBUG_VAR_2(std::to_string(tobs), idx);
        // PRX_DEBUG_VAR_2(t0, t1);
        const double t01{ std::max((tobs - t0) / (t1 - t0), 0.0) };

        graph.emplace_shared<mushr_x_async_observation_t>(k_X(idx, idx_offset), k_X(idx + 1, idx_offset), zt, t01,
                                                          config.cm_x_z);
      }
    }
  }
  // graph.addPrior(k_X(max_idx - 1, idx_offset), zt, config.cm_x_xdot);
}

void create_traj_graph(prx::param_loader& params, prx::trajectory_t& traj, prx::plan_t& plan,
                       std::shared_ptr<prx::system_group_t> sys_group, std::size_t idx, std::string observations_file,
                       std::string plans_file, gtsam::NonlinearFactorGraph& graph, gtsam::Values& values,
                       prx::fg::mushrTypes::ParamsUbarU& init_params, prx::space_point_t start_state)
{
  traj.clear();
  ObservedTrajectory observations{};

  read_observations(observations_file, observations);
  const double plan_t0{ read_ros_plan(plans_file, plan) };
  PRX_DEBUG_VAR_1(plan);
  increase_plan_to_match_trajectory(observations, plan_t0, plan, idx);
  PRX_DEBUG_VAR_1(plan.duration());

  const double dt0{ observations[1].first - observations[0].first };
  Vec(start_state) = Eigen::Vector<double, 6>::Zero();
  Vec(start_state).head(3) = observations[0].second;
  Vec(start_state).tail(3) = (observations[1].second - observations[0].second) / dt0;

  PRX_DEBUG_VAR_1(plan);
  sys_group->propagate(start_state, plan, traj);
  std::ios_base::openmode mode{ idx == 0 ? std::ofstream::trunc : std::ofstream::app };
  traj.to_file(prx::out_path + "mushr/multi_orig_traj.txt", mode);

  PRX_DEBUG_VAR_1(start_state);
  PRX_DEBUG_VAR_2(plan.size(), traj.size());
  PRX_DEBUG_VAR_2(plan.duration(), traj.duration());
  add_observations(graph, values, traj, plan, observations, init_params, idx);
}

int main(int argc, char* argv[])
{
  const std::string params_file{ "executables/factor_graphs/real_mushr_multi_traj.yaml" };
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

  prx::fg::mushrTypes::ParamsUbarU init_params{};

  ps->copy(init_params, params["params"].as<std::vector<double>>());
  ps->copy_from(init_params);
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;

  const std::string data_dir{ params["data_dir"].as<>() };
  std::vector<std::string> _observations{ params["observations"].as<std::vector<std::string>>() };
  std::vector<std::string> _plans{ params["plan"].as<std::vector<std::string>>() };
  std::vector<std::string> observations_in{};
  std::vector<std::string> plans_in{};

  auto transform_f = [&data_dir](const std::string& s) { return data_dir + s; };
  std::transform(_observations.begin(), _observations.end(),
                 _observations.begin(),  // write to the same location
                 transform_f);
  std::transform(_plans.begin(), _plans.end(),
                 _plans.begin(),  // write to the same location
                 transform_f);
  prx_assert(_observations.size() == _plans.size(), "observations and plans must be the same size");
  PRX_DEBUG_VAR_1(_plans[0]);
  const std::size_t observations_to_use{ _observations.size() / 2 };
  for (int i = 0; i < _observations.size(); ++i)
  {
    const double random_val{ prx::uniform_random(0.0, 1.0) };
    if (random_val > 0.5)
    {
      observations_in.push_back(_observations[i]);
      plans_in.push_back(_plans[i]);
    }
  }

  PRX_DEBUG_VAR_1(plans_in.size());
  std::vector<prx::space_point_t> start_states{};  // plans_in.size(), sys_group->get_state_space()->make_point());
  std::vector<prx::plan_t> plans(plans_in.size(), cs);
  prx::trajectory_t traj{ ss };
  // prx::space_point_t start_state{ ss->make_point() };

  for (int i = 0; i < plans_in.size(); ++i)
  {
    start_states.push_back(ss->make_point());
    create_traj_graph(params, traj, plans[i], sys_group, i, observations_in[i], plans_in[i], graph, values, init_params,
                      start_states[i]);
    PRX_DEBUG_VAR_1(start_states[i]);
  }
  using PositiveVecFactor = prx::fg::partial_positive_vector_factor_t<ParamsUbarU::RowsAtCompileTime>;
  auto pvm = gtsam::noiseModel::Isotropic::Sigma(ParamsUbarU::RowsAtCompileTime, 1e-5);
  graph.emplace_shared<PositiveVecFactor>(PositiveVecFactor::Vector(1, 0, 1, 1), k_Ps("dv"), pvm);
  values.insert(k_Ps("dv"), init_params);

  // plan.from_file(params["plan"].as<std::string>());

  gtsam::LevenbergMarquardtParams lm_params{ prx::fg::default_levenberg_marquardt_parameters() };

  lm_params.verbosityLMTranslator(gtsam::LevenbergMarquardtParams::SILENT);
  lm_params.setMaxIterations(params["LevenbergMarquardt/max_iters"].as<int>());
  // lm_params.setMaxIterations(1);
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
    auto factor3 = boost::dynamic_pointer_cast<prx::fg::mushr_x_async_observation_t>(factor);
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
    //   ofs << "mushr_x_async_observation_t ";
    //   factor3->eval_to_stream(results, ofs);
    // }
  }

  const prx::fg::mushrTypes::ParamsUbarU params_out{ results.at<prx::fg::mushrTypes::ParamsUbarU>(k_Ps("dv")) };

  PRX_DEBUG_VAR_1(params_out.transpose());
  printf("[%.4f, %.4f, %.4f, %.4f]\n", params_out[0], params_out[1], params_out[2], params_out[3]);
  // PRX_DEBUG_VAR_1(plans_in.size());
  ps->copy_from(params_out);

  prx::trajectory_t res_traj{ ss };
  for (int i = 0; i < plans_in.size(); ++i)
  {
    // PRX_DEBUG_VAR_1(plans[i]);
    res_traj.clear();
    std::ios_base::openmode mode{ i == 0 ? std::ofstream::trunc : std::ofstream::app };

    // PRX_DEBUG_VAR_1(start_states[i]);
    // PRX_DEBUG_VAR_1(plans[i]);
    sys_group->propagate(start_states[i], plans[i], res_traj);
    // PRX_DEBUG_VAR_1(res_traj);
    res_traj.to_file(prx::out_path + "mushr/multi_res_traj.txt", mode);
  }

  ofs.close();
  PRX_DEBUG_VAR_1(out_filename);
  return 0;
}