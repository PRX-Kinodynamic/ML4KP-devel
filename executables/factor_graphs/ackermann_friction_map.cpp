#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>
#include <memory>
#include <functional>
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
#include "prx/utilities/general/constants.hpp"
#include "prx/utilities/general/range.hpp"
#include "prx/utilities/geometry/regular_grid.hpp"
#include "prx/utilities/math/first_order_derivative.hpp"
#include "prx/utilities/general/csv_reader.hpp"

#include "prx/visualization/three_js_group.hpp"

#include "prx/factor_graphs/defs.hpp"
#include "prx/factor_graphs/factors/friction_fusion_factor.hpp"
#include "prx/factor_graphs/factors/mj_friction_propagation.hpp"
#include "prx/factor_graphs/factors/parameter_fusion_factor.hpp"
#include "prx/factor_graphs/factors/positive_vector_factor.hpp"
#include "prx/factor_graphs/factors/state_prior.hpp"
#include "prx/factor_graphs/utilities/constants.hpp"
#include "prx/factor_graphs/utilities/fg_logger.hpp"
#include "prx/factor_graphs/utilities/utilities_functions.hpp"
#include "prx/factor_graphs/utilities/formatter.hpp"
#include "prx/factor_graphs/utilities/friction_map.hpp"
#include "prx/factor_graphs/factors/ackermann_factors.hpp"
#include "prx/factor_graphs/utilities/friction_map.hpp"

#include "prx/mujoco/mj_simulator.hpp"
#include "prx/mujoco/plants/mj_friction_plant.hpp"
#include <gtsam/nonlinear/Marginals.h>
#include "friction_maps.hpp"
using namespace prx;
using namespace prx::fg;

const int XMAX{ 20 };
const int YMAX{ 20 };
const int GRID_DIVISIONS{ 10 };
const int BASIS_DIM{ (GRID_DIVISIONS + 1) * (GRID_DIVISIONS + 1) };

const Eigen::Index TH_DIM{ 1 };
// const Eigen::Index StateDim{ Eigen::Dynamic };
const double WHEEL_DISTANCE{ 0.115 * 2.0 };
const double MASS{ 0.498952 + 3.542137 + 0.1 };

using friction_vector_t = Eigen::Vector<double, TH_DIM>;
using Friction = Eigen::Vector<double, TH_DIM>;  // Todo: Consolidate
using basis_vector_t = Eigen::Vector<double, 4>;

using Q = typename fg::ackermann::Q;
using Qdot = typename fg::ackermann::Qdot;
using Qdotdot = typename fg::ackermann::Qdotdot;
using U = typename fg::ackermann::U;
using Force = typename fg::ackermann::Force;
using ModelParams = typename fg::ackermann::ModelParams;
using EnvironmentParams = typename fg::ackermann::EnvironmentParams;
using Z_Q = typename fg::ackermann::Qz;  // Observation on Q

// using Theta = typename PropagationMj::Theta;
using Position = Eigen::Vector<double, 2>;

using Weights = Eigen::Vector<double, 4>;
using Position = Eigen::Vector<double, 2>;
using LocalBasis = Eigen::Vector<double, 4>;
using BasisPosition = Eigen::Matrix<double, 4, 2>;

using LocalFusionFactor = prx::fg::friction_local_fusion_factor_t<TH_DIM, Z_Q, BasisPosition>;
using BasisFrictionFactor = prx::fg::friction_local_fusion_factor_t<TH_DIM, Z_Q, BasisPosition>;

using NoiseModels = typename std::unordered_map<std::string, gtsam::noiseModel::Base::shared_ptr>;
// using SymbolPositions = typename std::unordered_map<prx::prx_symbol_t, Position>;
using Graph = gtsam::NonlinearFactorGraph;
using Values = gtsam::Values;
// const prx_symbol_t k_params{ symbol_factory_t::create_hashed_symbol("Theta", 0) };

const Q q_init{ Q::Zero() };
const Qdot qdot_init{ ::Qdot::Zero() };
const Qdotdot qdotdot_init{ Qdotdot::Zero() };
const U u_init{ U::Zero() };
const Force force_init{ Force::Zero() };
EnvironmentParams env_params_init{ EnvironmentParams::Ones() };
double initial_friction = 0.0;
NoiseModels noise_models;

using namespace std::placeholders;  // for _1, _2, _3...
auto symbol_U = [](std::size_t idx) { return symbol_factory_t::create_hashed_symbol("U", idx); };
auto symbol_Q = [](std::size_t idx) { return symbol_factory_t::create_hashed_symbol("X", idx); };
auto symbol_Qdot = [](std::size_t idx) { return symbol_factory_t::create_hashed_symbol("Xdot", idx); };
auto symbol_Qdotdot = [](std::size_t idx) { return symbol_factory_t::create_hashed_symbol("Xdotdot", idx); };
auto symbol_ModelParams = [](std::size_t idx) { return symbol_factory_t::create_hashed_symbol("ThetaModel", idx); };
auto symbol_EnvParams = [](std::size_t idx) { return symbol_factory_t::create_hashed_symbol("ThetaEnvironment", idx); };
auto symbol_Force = [](std::size_t idx) { return symbol_factory_t::create_hashed_symbol("F", idx); };
auto symbol_Zq = [](std::size_t idx) { return symbol_factory_t::create_hashed_symbol("Z", idx); };

void create_ackermann_at_idx_fg(Graph& graph, Values& values, const std::size_t idx)
{
  const prx_symbol_t k_u{ symbol_U(idx) };
  const prx_symbol_t k_q0{ symbol_Q(idx) };
  const prx_symbol_t k_q1{ symbol_Q(idx + 1) };
  const prx_symbol_t k_qdot{ symbol_Qdot(idx) };
  const prx_symbol_t k_params_m{ symbol_ModelParams(0) };
  const prx_symbol_t k_params_e{ symbol_EnvParams(0) };
  const prx_symbol_t k_qdotdot{ symbol_Qdotdot(idx) };

  const prx_symbol_t k_force{ symbol_Force(idx) };

  graph.add(ackermann_q_qdot_u_t(k_q0, k_q1, k_qdot, k_u, noise_models["f1"]));
  graph.add(ackermann_q_qdot_qdotdot_t(k_qdot, k_qdotdot, k_q0, noise_models["f2"], WHEEL_DISTANCE));
  graph.add(ackermann_qdotdot_force_q_t(k_qdotdot, k_force, k_q0, k_params_m, k_params_e, noise_models["f3"],
                                        WHEEL_DISTANCE, MASS));

  // values.insert(k_u, u_init);
  values.insert(k_q0, q_init);
  values.insert(k_qdot, qdot_init);
  values.insert(k_qdotdot, qdotdot_init);
  values.insert(k_force, force_init);
  values.insert_or_assign(k_params_e, env_params_init);
}

template <typename BasisGrid, typename FrictionsGrid, typename SymbolPositions, typename VisitedGrid>
void friction_map_add(const std::size_t idx, const Z_Q& zq, Graph& graph, Values& values, const double length,
                      BasisGrid& basis_grid, FrictionsGrid& frictions_grid, NoiseModels& noise_models,
                      SymbolPositions& symbol_positions, VisitedGrid& visited_grid)
{
  const prx_symbol_t k_q{ symbol_Q(idx) };
  const prx_symbol_t k_zq{ symbol_Zq(idx) };
  // const prx_symbol_t k_params_m{ symbol_ModelParams(idx) };
  const prx_symbol_t k_params_e{ symbol_EnvParams(0) };

  const Position z_xy{ zq.head(2) };
  symbol_positions[k_params_e] = z_xy;
  visited_grid(z_xy[0], z_xy[1])[0] = 1;

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

  for (int i = 0; i < 10; ++i)
  {
    // Params
    values.insert_or_assign(k_params_e, EnvironmentParams{ current_param_value });
    graph.add(BasisFrictionFactor(noise_models["small_basis"], symbol_EnvParams(idx), basis_symbol_0, basis_symbol_1,
                                  basis_symbol_2, basis_symbol_3, zq, basis_positions, length, TH_DIM, 1));
  }
  values.insert_or_assign(basis_symbol_0, (Eigen::VectorXd(1) << local_basis[0]).finished());
  values.insert_or_assign(basis_symbol_1, (Eigen::VectorXd(1) << local_basis[1]).finished());
  values.insert_or_assign(basis_symbol_2, (Eigen::VectorXd(1) << local_basis[2]).finished());
  values.insert_or_assign(basis_symbol_3, (Eigen::VectorXd(1) << local_basis[3]).finished());
  // basis_used[basis_symbol_0] = basis_positions.row(0);
  // basis_used[basis_symbol_1] = basis_positions.row(1);
  // basis_used[basis_symbol_2] = basis_positions.row(2);
  // basis_used[basis_symbol_3] = basis_positions.row(3);
  // graph.add(PositiveVectorFactor(basis_symbol.first, _noise_models["positive_friction"]));
}

template <typename BasisGrid, typename FrictionsGrid, typename VisitedGrid>
void to_files(gtsam::Values& values, const std::string prefix, double length, BasisGrid& basis_grid,
              VisitedGrid& visited_grid, FrictionsGrid& frictions_grid)
{
  const std::string fm_out_dir = prx::out_path + "friction_maps/" + prefix + "_";
  logger_t log_frictionmap(fm_out_dir + "fmbasis_idd_friction_map.txt");

  basis_grid.to_file(fm_out_dir + "frictions_grid_mj.txt", [&](const prx::prx_symbol_t& s) {
    if (values.exists(s))
      return values.at<Friction>(s)[0];
    else
      return -1.0;
  });
  visited_grid.to_file(fm_out_dir + "visited_grid_mj.txt", [&](const Friction& v) { return v.transpose(); });

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
}

int main(int argc, char** argv)
{
  const std::string params_file{ "executables/factor_graphs/ackermann_friction_map.yaml" };
  param_loader params(params_file, argc, argv);

  init_random(params["random_seed"].as<int>());

  const std::vector<std::pair<double, double>> env_bounds{ std::make_pair(-XMAX, XMAX), std::make_pair(-YMAX, YMAX) };
  prx::regular_grid_t<friction_vector_t, 2> frictions_grid{ env_bounds, GRID_DIVISIONS };
  prx::regular_grid_t<friction_vector_t, 2> visited_grid{ env_bounds, GRID_DIVISIONS };

  prx::regular_grid_t<prx::prx_symbol_t, 2> basis_grid{ env_bounds, GRID_DIVISIONS };
  std::unordered_map<prx::prx_symbol_t, Position> symbol_positions;

  initial_friction = params["initial_friction"].as<double>();
  env_params_init = friction_vector_t::Ones(TH_DIM) * initial_friction;
  const Eigen::VectorXd initial_friction_vec{ friction_vector_t::Ones(TH_DIM) * initial_friction };

  frictions_grid.populate_grid(env_params_init);
  visited_grid.populate_grid(friction_vector_t::Zero());

  const double Length_0{ frictions_grid.get_cell_length(0) };

  Graph graph;
  Values values;
  gtsam::LevenbergMarquardtParams lm_params{ fg::utilities::default_levenberg_marquardt_parameters() };
  lm_params.setUseFixedLambdaFactor(true);
  lm_params.setMaxIterations(10000);
  lm_params.setRelativeErrorTol(1e-10);
  lm_params.setAbsoluteErrorTol(1e-10);
  lm_params.setlambdaUpperBound(1e64);
  lm_params.setVerbosityLM("SUMMARY");

  // noise_models["state_space"] = gtsam::noiseModel::Diagonal::Sigmas(noise);
  noise_models["Z_prior"] = gtsam::noiseModel::Isotropic::Sigma(fg::ackermann::DimQz, 1e-1);
  noise_models["u_prior"] = gtsam::noiseModel::Isotropic::Sigma(fg::ackermann::DimU, 1e-5);
  noise_models["q_prior"] = gtsam::noiseModel::Isotropic::Sigma(fg::ackermann::DimQ, 1e-3);
  noise_models["f1"] = gtsam::noiseModel::Isotropic::Sigma(fg::ackermann::DimQ, 1e-0);
  noise_models["f2"] = gtsam::noiseModel::Isotropic::Sigma(fg::ackermann::DimQdot, 1e-0);
  noise_models["f3"] = gtsam::noiseModel::Isotropic::Sigma(fg::ackermann::DimQdotdot, 1e-0);
  noise_models["fZ"] = gtsam::noiseModel::Isotropic::Sigma(fg::ackermann::DimQz, 1e-3);

  std::string traj_file{ params["traj_file"].as<>() };
  std::cout << traj_file << std::endl;
  prx::utilities::csv_reader_t reader(traj_file);

  const std::size_t T{ 4'000 };
  graph.addPrior(symbol_Q(0), q_init, noise_models["q_prior"]);
  fg::ackermann::U u_rand{ fg::ackermann::U::Random() };
  u_rand[0] = 1;
  u_rand[1] = 0.52;
  std::vector<fg::ackermann::U> ctrls = {
    { 0.933466, -0.001465 }, { 0.944210, 0.010173 }, { 0.942091, -0.062425 }, { 0.804396, 0.042345 }
  };

  for (std::size_t i = 0; i < T; ++i)
  {
    create_ackermann_at_idx_fg(graph, values, i);

    if (i % 1'000 == 0)
    {
      u_rand = ctrls[0];
      ctrls.erase(ctrls.begin());
    }
    const prx_symbol_t k_u{ symbol_U(i) };
    graph.addPrior(k_u, u_rand, noise_models["u_prior"]);
    values.insert_or_assign(k_u, u_rand);
  }
  values.insert(symbol_Q(T), q_init);

  const fg::ackermann::ModelParams model_params_init{ 0.01 };
  values.insert(symbol_ModelParams(0), model_params_init);

  std::size_t idx{ 0 };
  while (reader.has_next_line())
  {
    auto line = reader.next_line<double>();
    if (line.size() == 0)
      continue;
    const prx_symbol_t k_q{ symbol_Q(idx) };
    const prx_symbol_t k_zq{ symbol_Zq(idx) };
    const Z_Q q_z{ line[0], line[1], line[2] };

    graph.add(ackermann_q_observation_t(k_zq, k_q, noise_models["fZ"]));
    graph.addPrior(k_zq, q_z, noise_models["Z_prior"]);
    values.insert_or_assign(k_zq, q_z);
    // friction_map_add(idx, q_z, graph, values, Length_0, basis_grid, frictions_grid, noise_models, symbol_positions,
    //                  visited_grid);
    idx += 10;
  }

  symbol_factory_t::symbols_to_file();
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
  logger_t logger(out_path + "ackermann_friction_map.log");
  gtsam::Values results = fg::utilities::optimize_and_log(optimizer, lm_params, logger, 0);

  std::function<std::tuple<bool, Position>(const gtsam::Values&, const gtsam::Key&)> variables_positions =
      [&](const gtsam::Values& values, const gtsam::Key& key)  // no-lint
  {
    bool bool_res{ false };
    Position pos_res{ Position::Zero() };
    if (symbol_positions.count(key) > 0)
    {
      bool_res = true;
      pos_res = 1000 * symbol_positions[key];
    }

    return std::make_tuple(bool_res, pos_res);
  };
  prx::fg::create_gml_file(graph, results, prx::out_path + "friction_maps/ackermann_friction_map.gml",
                           variables_positions);
  prx::fg::utilities::values_to_file<basis_vector_t>(results, prx::out_path +
                                                                  "friction_maps/"
                                                                  "ackermann_friction_map.txt");
  prx::logger_t fm_log{ prx::out_path + "idd_friction_map.txt" };
  prx::friction_map::compute_friction_map<basis_vector_t>(frictions_grid, visited_grid, fm_log,
                                                          prx::linspace<double>(-XMAX, XMAX, 20),
                                                          prx::linspace<double>(-YMAX, YMAX, 20));
  to_files(results, "ackermann", Length_0, basis_grid, visited_grid, frictions_grid);
  return 0;
}