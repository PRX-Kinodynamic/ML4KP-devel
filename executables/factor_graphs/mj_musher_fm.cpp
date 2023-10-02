#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>

#include <boost/algorithm/string/join.hpp>
#include <boost/optional.hpp>

#include "prx/planning/loaders/planner_loader.hpp"
#include "prx/planning/planners/rrt.hpp"
#include "prx/planning/planner_functions/planner_functions.hpp"
#include "prx/planning/planner_statistics.hpp"
#include "prx/planning/world_model.hpp"

#include "prx/simulation/controllers/custom_controller.hpp"
#include "prx/simulation/general/condition_check.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/simulation/plants/types/noisy_plant.hpp"
#include "prx/simulation/playback/trajectory.hpp"
#include "prx/simulation/system_factory.hpp"
#include "prx/simulation/system_group.hpp"

#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/csv_reader.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/utilities/general/range.hpp"
#include "prx/utilities/geometry/regular_grid.hpp"

#include "prx/visualization/three_js_group.hpp"

#include "prx/factor_graphs/defs.hpp"
#include "prx/factor_graphs/utilities/fg_logger.hpp"
#include "prx/factor_graphs/planning/initialization_trajs_fg.hpp"
#include "prx/factor_graphs/factors/parameter_fusion_factor.hpp"
#include "prx/factor_graphs/utilities/fg_to_csv.hpp"
#include "prx/factor_graphs/factors/euclidian_distance_factor.hpp"
#include "prx/factor_graphs/factors/ackermann_factors.hpp"
#include "prx/factor_graphs/factors/positive_vector_factor.hpp"
#include "prx/factor_graphs/factors/state_prior.hpp"
#include "prx/factor_graphs/graphs/ilqr.hpp"
#include "prx/factor_graphs/utilities/constants.hpp"
#include "prx/factor_graphs/utilities/formatter.hpp"
#include "prx/factor_graphs/utilities/utilities_functions.hpp"

#include "prx/factor_graphs/factors/friction_fusion_factor.hpp"
#include "prx/factor_graphs/factors/mj_friction_propagation.hpp"
#include "prx/factor_graphs/utilities/friction_map.hpp"

#include "prx/mujoco/mj_simulator.hpp"

#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/linear/JacobianFactor.h>
using namespace prx;
using namespace prx::fg;
using namespace prx::utilities;

std::unordered_map<std::string, gtsam::noiseModel::Base::shared_ptr> noise_models;

const int GRID_DIVISIONS{ 10 };
const int BASIS_DIM{ (GRID_DIVISIONS + 1) * (GRID_DIVISIONS + 1) };
const double WHEEL_DISTANCE{ 0.115 * 2.0 };
const double MASS{ 0.498952 + 3.542137 + 0.1 };
const Eigen::Index TH_DIM{ 1 };

using friction_vector_t = Eigen::Vector<double, TH_DIM>;
using Friction = Eigen::Vector<double, TH_DIM>;  // Todo: Consolidate
using basis_vector_t = Eigen::Vector<double, 4>;
using SF = prx::symbol_factory_t;

const prx_symbol_t k_params_m{ symbol_factory_t::create_hashed_symbol("ThetaModel", 0) };

using Q = typename fg::ackermann::Q;
using Qdot = typename fg::ackermann::Qdot;
using Qdotdot = typename fg::ackermann::Qdotdot;
using U = typename fg::ackermann::U;
using Force = typename fg::ackermann::Force;
using ModelParams = typename fg::ackermann::ModelParams;
using EnvironmentParams = typename fg::ackermann::EnvironmentParams;
using Z_Q = typename fg::ackermann::Qz;
using Duration = typename fg::ackermann::Duration;

const U u_init{ fg::ackermann::U::Zero() };
const Q q_init{ fg::ackermann::Q::Zero() };
const Qdot qdot_init{ fg::ackermann::Qdot::Zero() };
const Qdotdot qdotdot_init{ fg::ackermann::Qdotdot::Zero() };
const Force force_init{ fg::ackermann::Force::Zero() };
double initial_friction = 0.0;

using Position = Eigen::Vector<double, 2>;
using Weights = Eigen::Vector<double, 4>;
using Position = Eigen::Vector<double, 2>;
using LocalBasis = Eigen::Vector<double, 4>;
using BasisPosition = Eigen::Matrix<double, 4, 2>;

using LocalFusionFactor = prx::fg::friction_local_fusion_factor_t<TH_DIM, Z_Q, BasisPosition>;
using BasisFrictionFactor = prx::fg::friction_local_fusion_factor_t<TH_DIM, Z_Q, BasisPosition>;

using Graph = gtsam::NonlinearFactorGraph;
using NoiseModels = typename std::unordered_map<std::string, gtsam::noiseModel::Base::shared_ptr>;
using Values = gtsam::Values;

auto symbol_U = [](std::size_t ti, std::size_t xi) { return SF::create_hashed_symbol("U^{", ti, "}_{", xi, "}"); };
auto symbol_Q = [](std::size_t ti, std::size_t xi) { return SF::create_hashed_symbol("X^{", ti, "}_{", xi, "}"); };
auto symbol_Qdot = [](std::size_t ti, std::size_t xi) {
  return SF::create_hashed_symbol("Xdot^{", ti, "}_{", xi, "}");
};
auto symbol_Qdotdot = [](std::size_t ti, std::size_t xi) {
  return SF::create_hashed_symbol("Xdotdot^{", ti, "}_{", xi, "}");
};
auto symbol_ModelParams = [](std::size_t idx) { return SF::create_hashed_symbol("ThetaModel", idx); };
auto symbol_EnvParams = [](std::size_t ti, std::size_t xi) {
  return SF::create_hashed_symbol("Th^{", ti, "}_{", xi, "}");
};
auto symbol_Force = [](std::size_t ti, std::size_t xi) { return SF::create_hashed_symbol("F^{", ti, "}_{", xi, "}"); };
auto symbol_Zq = [](std::size_t ti, std::size_t xi) { return SF::create_hashed_symbol("Z^{", ti, "}_{", xi, "}"); };
auto symbol_basis = [](std::size_t idx) { return SF::create_hashed_symbol("TH^{B}_{", idx, "}"); };

fg::ackermann::EnvironmentParams environment_params_init{ 1.0 };

void values_to_file(const std::string& filename, const std::size_t N, gtsam::Values& values,
                    const std::size_t tot_trajs)
{
  logger_t results_logger(filename);
  std::size_t idx{ 0 };
  for (std::size_t ti = 0; ti < tot_trajs; ++ti)
  {
    for (idx = 0; idx < N; ++idx)
    {
      const auto r_U{ values.at<fg::ackermann::U>(symbol_U(ti, idx)) };
      const auto r_Q{ values.at<fg::ackermann::Q>(symbol_Q(ti, idx)) };
      const auto r_Qdot{ values.at<fg::ackermann::Qdot>(symbol_Qdot(ti, idx)) };
      const auto r_Qdotdot{ values.at<fg::ackermann::Qdotdot>(symbol_Qdotdot(ti, idx)) };
      const auto r_Force{ values.at<fg::ackermann::Force>(symbol_Force(ti, idx)) };
      const auto r_EnvParams{ values.at<fg::ackermann::EnvironmentParams>(symbol_EnvParams(ti, idx)) };

      results_logger("U", idx, r_U.transpose());
      results_logger("Q", idx, r_Q.transpose());
      results_logger("Qdot", idx, r_Qdot.transpose());
      results_logger("Qdotdot", idx, r_Qdotdot.transpose());
      results_logger("Force", idx, r_Force.transpose());
      results_logger("EnvParams", idx, r_EnvParams.transpose());
    }
  }
}

template <typename ValueType, typename SymbolPositions, typename SymbolFunction>
void update_symbol_positions(SymbolPositions& symbol_positions, const std::size_t N, const std::size_t tot_trajs,
                             gtsam::Values& values, SymbolFunction& symbol_function, Eigen::Vector2d offset)
{
  for (std::size_t ti = 0; ti < tot_trajs; ++ti)
  {
    for (std::size_t idx = 0; idx < N; ++idx)
    {
      const gtsam::Key key{ symbol_function(ti, idx) };
      const ValueType vi{ values.at<ValueType>(key) };
      symbol_positions[key] = vi.head(2) + offset;
    }
  }
}

void create_ackermann_at_idx_fg(gtsam::NonlinearFactorGraph& graph, gtsam::Values& values, const std::size_t idx,
                                const std::size_t traj_id, const double h)
{
  const prx_symbol_t k_u{ symbol_U(traj_id, idx) };
  const prx_symbol_t k_q0{ symbol_Q(traj_id, idx) };
  const prx_symbol_t k_q1{ symbol_Q(traj_id, idx + 1) };
  const prx_symbol_t k_qdot{ symbol_Qdot(traj_id, idx) };
  const prx_symbol_t k_qdotdot{ symbol_Qdotdot(traj_id, idx) };

  const prx_symbol_t k_force{ symbol_Force(traj_id, idx) };
  const prx_symbol_t k_params_e{ symbol_EnvParams(traj_id, idx) };

  graph.add(ackermann_q_qdot_u_t(k_q0, k_q1, k_qdot, k_u, noise_models["f1"], h));
  graph.add(ackermann_q_qdot_qdotdot_t(k_qdot, k_qdotdot, k_q0, noise_models["f2"], WHEEL_DISTANCE, h));
  graph.add(ackermann_qdotdot_force_q_t(k_qdotdot, k_force, k_q0, k_params_m, k_params_e, noise_models["f3"],
                                        WHEEL_DISTANCE, MASS, h));
  // ackermann_qdotdot_force_q_t
  // values.insert(k_u, u_init);
  // values.insert_or_assign(k_q0, q_init);
  // values.insert_or_assign(k_q1, q_init);
  values.insert(k_qdot, qdot_init);
  values.insert(k_qdotdot, qdotdot_init);
  values.insert(k_force, force_init);
  values.insert_or_assign(k_params_e, environment_params_init);
}

template <typename BasisGrid, typename FrictionsGrid, typename SymbolPositions, typename VisitedGrid>
void friction_map_add(const std::size_t idx, const std::size_t traj_idx, const Z_Q& zq, Graph& graph, Values& values,
                      const double length, BasisGrid& basis_grid, FrictionsGrid& frictions_grid,
                      NoiseModels& noise_models, SymbolPositions& symbol_positions, VisitedGrid& visited_grid)
{
  const prx_symbol_t k_q{ symbol_Q(traj_idx, idx) };
  const prx_symbol_t k_zq{ symbol_Zq(traj_idx, idx) };
  // const prx_symbol_t k_params_m{ symbol_ModelParams(idx) };
  const prx_symbol_t k_params_e{ symbol_EnvParams(traj_idx, idx) };

  const Position z_xy{ zq.head(2) };
  symbol_positions[k_params_e] = z_xy + Eigen::Vector2d(0, -0.1);
  visited_grid(z_xy[0], z_xy[1])[0] = 1;
  // PRX_DEBUG_VAR_2(z_xy.transpose(), visited_grid(z_xy[0], z_xy[1])[0]);

  const Position position_0{ z_xy };
  const Position position_1{ z_xy + Position(0, length) };
  const Position position_2{ z_xy + Position(length, 0) };
  const Position position_3{ z_xy + Position(length, length) };

  const prx::prx_symbol_t basis_symbol_0{ basis_grid(position_0[0], position_0[1]) };
  const prx::prx_symbol_t basis_symbol_1{ basis_grid(position_1[0], position_1[1]) };
  const prx::prx_symbol_t basis_symbol_2{ basis_grid(position_2[0], position_2[1]) };
  const prx::prx_symbol_t basis_symbol_3{ basis_grid(position_3[0], position_3[1]) };
  BasisPosition basis_positions;
  basis_positions.row(0) = frictions_grid.template unmap<Position>(position_0[0], position_0[1]);
  basis_positions.row(1) = frictions_grid.template unmap<Position>(position_1[0], position_1[1]);
  basis_positions.row(2) = frictions_grid.template unmap<Position>(position_2[0], position_2[1]);
  basis_positions.row(3) = frictions_grid.template unmap<Position>(position_3[0], position_3[1]);
  symbol_positions[basis_symbol_0] = basis_positions.row(0);
  symbol_positions[basis_symbol_1] = basis_positions.row(1);
  symbol_positions[basis_symbol_2] = basis_positions.row(2);
  symbol_positions[basis_symbol_3] = basis_positions.row(3);

  Weights init_weight{ LocalFusionFactor::compute_weight(position_0, length, basis_positions) };
  LocalBasis local_basis;
  std::vector<Position> positions = { position_0, position_1, position_2, position_3 };
  for (int i = 0; i < 4; ++i)
  {
    if (values.exists(basis_grid(positions[i][0], positions[i][1])))
    {
      local_basis[i] = values.at<Friction>(basis_grid(positions[i][0], positions[i][1]))[0];
    }
    else
    {
      local_basis[i] = initial_friction;
    }
  }
  auto current_param_value = init_weight.dot(local_basis);
  // PRX_DEBUG_VAR_2(prx::symbol_factory_t::formatter(k_params_e), local_basis.transpose());
  // PRX_DEBUG_VAR_1(prx::symbol_factory_t::formatter(k_params_e));
  // PRX_DEBUG_VAR_1(init_weight.transpose());
  // PRX_DEBUG_VAR_1(local_basis.transpose());
  // PRX_DEBUG_VAR_1(current_param_value);

  // Params
  values.insert_or_assign(k_params_e, EnvironmentParams{ current_param_value });
  graph.add(BasisFrictionFactor(noise_models["small_basis"], k_params_e, basis_symbol_0, basis_symbol_1, basis_symbol_2,
                                basis_symbol_3, zq, basis_positions, length, TH_DIM, 1));
  values.insert_or_assign(basis_symbol_0, (Eigen::VectorXd(1) << local_basis[0]).finished());
  values.insert_or_assign(basis_symbol_1, (Eigen::VectorXd(1) << local_basis[1]).finished());
  values.insert_or_assign(basis_symbol_2, (Eigen::VectorXd(1) << local_basis[2]).finished());
  values.insert_or_assign(basis_symbol_3, (Eigen::VectorXd(1) << local_basis[3]).finished());
}

template <typename BasisGrid, typename FrictionsGrid, typename VisitedGrid>
void to_files(gtsam::Values& values, const std::string prefix, double length, BasisGrid& basis_grid,
              VisitedGrid& visited_grid, FrictionsGrid& frictions_grid)
{
  const std::string fm_out_dir = prx::out_path + "friction_maps/" + prefix + "_";
  const std::string log_frictionmap_file{ fm_out_dir + "mj_musher_friction_map.txt" };
  const std::string basis_grid_file{ fm_out_dir + "frictions_grid_mj.txt" };
  const std::string visited_grid_file{ fm_out_dir + "visited_grid_mj.txt" };
  logger_t log_frictionmap(log_frictionmap_file);

  basis_grid.to_file(basis_grid_file, [&](const prx::prx_symbol_t& s) {
    if (values.exists(s))
      return values.at<Friction>(s)[0];
    else
      return -1.0;
  });
  visited_grid.to_file(visited_grid_file, [&](const Friction& v) { return v.transpose(); });

  auto x_bounds = visited_grid.bounds(0);
  auto y_bounds = visited_grid.bounds(1);

  auto X = prx::linspace<double>(x_bounds.first, x_bounds.second, 100);
  auto Y = prx::linspace<double>(y_bounds.first, y_bounds.second, 100);
  for (auto x : X)
  {
    for (auto y : Y)
    {
      if (visited_grid(x, y)[0] > 0)
      {
        Position position_0(x, y);
        Position position_1(x, y + length);
        Position position_2(x + length, y);
        Position position_3(x + length, y + length);
        BasisPosition basis_positions;
        basis_positions.row(0) = frictions_grid.template unmap<Eigen::Vector2d>(position_0[0], position_0[1]);
        basis_positions.row(1) = frictions_grid.template unmap<Eigen::Vector2d>(position_1[0], position_1[1]);
        basis_positions.row(2) = frictions_grid.template unmap<Eigen::Vector2d>(position_2[0], position_2[1]);
        basis_positions.row(3) = frictions_grid.template unmap<Eigen::Vector2d>(position_3[0], position_3[1]);
        auto weight = LocalFusionFactor::compute_weight(position_0, length, basis_positions);

        auto basis_0 = values.at<Friction>(basis_grid(position_0[0], position_0[1]));
        auto basis_1 = values.at<Friction>(basis_grid(position_1[0], position_1[1]));
        auto basis_2 = values.at<Friction>(basis_grid(position_2[0], position_2[1]));
        auto basis_3 = values.at<Friction>(basis_grid(position_3[0], position_3[1]));
        Eigen::Vector4d basis(basis_0[0], basis_1[0], basis_2[0], basis_3[0]);

        auto friction_at_xy = weight.dot(basis);
        log_frictionmap(x, y, friction_at_xy);
      }
      else
      {
        log_frictionmap(x, y, 1.0);
      }
    }
  }

  PRX_DEBUG_VAR_1(log_frictionmap_file);
  PRX_DEBUG_VAR_1(basis_grid_file);
  PRX_DEBUG_VAR_1(visited_grid_file);
}

int main(int argc, char* argv[])
{
  const std::string params_file{ "executables/factor_graphs/mj_musher_fm.yaml" };
  prx::param_loader params(params_file, argc, argv);
  simulation_step = params["simulation_step"].as<double>();
  std::unordered_map<prx::prx_symbol_t, Position> symbol_positions;

  std::function<std::tuple<bool, Position>(const gtsam::Values&, const gtsam::Key&)> variables_positions =
      [&](const gtsam::Values& values, const gtsam::Key& key)  // no-lint
  {
    bool bool_res{ false };
    Position pos_res{ Position::Zero() };
    if (symbol_positions.count(key) > 0)
    {
      bool_res = true;
      pos_res = symbol_positions[key];
    }

    return std::make_tuple(bool_res, pos_res);
  };
  noise_models["u_prior"] = gtsam::noiseModel::Isotropic::Sigma(fg::ackermann::DimU, 1e-5);
  noise_models["q_prior"] = gtsam::noiseModel::Isotropic::Sigma(fg::ackermann::DimQ, 1e-3);
  noise_models["qdot_prior"] = gtsam::noiseModel::Isotropic::Sigma(fg::ackermann::DimQdot, 1e-3);
  noise_models["f1"] = gtsam::noiseModel::Isotropic::Sigma(fg::ackermann::DimQ, 1e0);
  noise_models["f2"] = gtsam::noiseModel::Isotropic::Sigma(fg::ackermann::DimQdot, 1e0);
  noise_models["f3"] = gtsam::noiseModel::Isotropic::Sigma(fg::ackermann::DimQdotdot, 1e-2);
  noise_models["fZ"] = gtsam::noiseModel::Isotropic::Sigma(fg::ackermann::DimQz, 1e0);
  noise_models["small_basis"] = gtsam::noiseModel::Isotropic::Sigma(1, 1e0);

  const std::pair<double, double> X_bounds{ params["environment/bounds/x"].as<std::pair<double, double>>() };
  const std::pair<double, double> Y_bounds{ params["environment/bounds/y"].as<std::pair<double, double>>() };

  const std::vector<std::pair<double, double>> env_bounds{ X_bounds, Y_bounds };
  prx::regular_grid_t<friction_vector_t, 2> frictions_grid{ env_bounds, GRID_DIVISIONS };
  prx::regular_grid_t<friction_vector_t, 2> visited_grid{ env_bounds, GRID_DIVISIONS };

  prx::regular_grid_t<prx::prx_symbol_t, 2> basis_grid{ env_bounds, GRID_DIVISIONS };

  initial_friction = params["initial_friction"].as<double>();
  environment_params_init = friction_vector_t::Ones(TH_DIM) * initial_friction;

  std::size_t basis_idx{ 0 };
  std::function<prx::prx_symbol_t(const std::vector<double>&)> basis_grid_initializer =
      [&](const std::vector<double>&) {
        const prx::prx_symbol_t basis_symbol{ symbol_basis(basis_idx) };
        basis_idx++;
        return basis_symbol;
      };

  frictions_grid.populate_grid(environment_params_init);
  visited_grid.populate_grid(friction_vector_t::Zero());
  basis_grid.populate_grid(basis_grid_initializer);

  const double Length_0{ frictions_grid.get_cell_length(0) };

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  const fg::ackermann::ModelParams model_params_init{ 0.01 };
  values.insert(k_params_m, model_params_init);

  const std::string data_directory{ params["data_directory"].as<>() };

  const std::size_t tot_trajs{ 1 };
  // const std::size_t start{ params["data_to_process/first"].as<std::size_t>() };
  // const std::size_t end{ params["data_to_process/second"].as<std::size_t>() };
  // const std::size_t tot_trajs{ end - start };
  // for (int i = start; i < end; ++i)
  // {
  //   std::ostringstream ss;
  //   ss << std::setw(5) << std::setfill('0') << 12 << "\n";
  // }
  prx::utilities::csv_reader_t traj_reader(data_directory + "/observation/obs_00002.txt");
  prx::utilities::csv_reader_t plan_reader(data_directory + "/plans/plan_00002.txt");
  auto ctrl_step = plan_reader.next_line<double>();
  const U ctrl(ctrl_step[1], ctrl_step[2]);

  std::vector<Q> traj{};
  while (traj_reader.has_next_line())
  {
    auto line = traj_reader.next_line<double>();
    if (line.size() == 0)
      continue;
    // const Eigen::Quaternion<double> quat(line[3], line[4], line[5], line[6]);
    traj.emplace_back(line[0], line[1], line[3], 0, 0);
  }
  std::size_t idx{ 0 };
  std::size_t idx_traj{ 0 };
  for (idx = 0; idx < traj.size() - 1; idx++)
  {
    const Q q{ traj[idx] };
    const Z_Q zq{ q.head(3) };
    PRX_DEBUG_VAR_2(prx::symbol_factory_t::formatter(symbol_Q(idx_traj, idx)), q.transpose());

    graph.add(ackermann_q_observation_t(zq, symbol_Q(idx_traj, idx), noise_models["fZ"]));
    values.insert_or_assign(symbol_Q(idx_traj, idx), q);
    symbol_positions[symbol_Q(idx_traj, idx)] = zq.head(2);

    const double sim_step{ 0.1 };
    create_ackermann_at_idx_fg(graph, values, idx, tot_trajs, sim_step);
    graph.addPrior(symbol_U(idx_traj, idx), ctrl, noise_models["u_prior"]);
    values.insert_or_assign(symbol_U(idx_traj, idx), ctrl);
    symbol_positions[symbol_U(idx_traj, idx)] = zq.head(2) + Eigen::Vector2d(0.1, 0.1);
    friction_map_add(idx, idx_traj, zq, graph, values, Length_0, basis_grid, frictions_grid, noise_models,
                     symbol_positions, visited_grid);
  }
  values.insert_or_assign(symbol_Q(idx_traj, idx), traj[idx]);
  PRX_DEBUG_VAR_2(prx::symbol_factory_t::formatter(symbol_Q(idx_traj, idx)), traj[idx].transpose());

  symbol_factory_t::symbols_to_file();

  prx::fg::fg_to_csv<2>(graph, values, prx::out_path + "/mj_musher_fm_graph.txt", variables_positions);

  gtsam::LevenbergMarquardtParams lm_params{ fg::utilities::default_levenberg_marquardt_parameters() };
  lm_params.verbosityLMTranslator(gtsam::LevenbergMarquardtParams::SILENT);
  lm_params.setMaxIterations(50);
  // lm_params.setMaxIterations(10000);
  lm_params.setRelativeErrorTol(1e-8);
  lm_params.setAbsoluteErrorTol(1e-8);
  lm_params.setlambdaUpperBound(1e64);
  lm_params.print("lm_params");

  logger_t logger(out_path + "ackermann_fg.log");
  values_to_file(out_path + "mj_musher_fm_initvals.txt", traj.size() - 1, values, tot_trajs);
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
  gtsam::Values results = fg::utilities::optimize_and_log(optimizer, lm_params, logger, 0);

  values_to_file(out_path + "mj_musher_fm_results.txt", traj.size() - 1, results, tot_trajs);
  // update_symbol_positions<fg::ackermann::Q>(symbol_positions, traj.size() - 1, results, symbol_Q,
  //                                           Eigen::Vector2d::Zero());

  PRX_DEBUG_VAR_1(graph.size());
  PRX_DEBUG_VAR_1(results.size());
  prx::fg::fg_errors_to_csv(graph, results, prx::out_path + "/mj_musher_graph_errors.txt");
  graph.printErrors(results, "MJ-Musher ", prx::symbol_factory_t::formatter);
  to_files(results, "ackermann", Length_0, basis_grid, visited_grid, frictions_grid);
  prx::fg::fg_to_csv<2>(graph, results, prx::out_path + "/mj_musher_fm_result_graph.txt", variables_positions);

  prx::fg::create_gml_file(graph, results, prx::out_path + "friction_maps/ackermann_mj_musher_fm.gml",
                           variables_positions);

  return 0;
}