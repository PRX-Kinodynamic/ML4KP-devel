#include <algorithm>
#include <iostream>

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
#include "prx/factor_graphs/lie_groups/lie_ode_observation.hpp"
#include "prx/factor_graphs/lie_groups/se2.hpp"
#include "prx/factor_graphs/lie_groups/lie_integrator.hpp"
#include "prx/factor_graphs/factors/prx_propagation_factor.hpp"
#include "prx/factor_graphs/factors/constraint_factor.hpp"
#include "prx/factor_graphs/utilities/fg_ilqr.hpp"

#include "prx/utilities/data_structures/regular_grid.hpp"

// #include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
// #include <gtsam/basis/FitBasis.h>
// #include <gtsam/basis/Chebyshev2.h>

using State = Eigen::Vector2d;
using Control = Eigen::Vector<double, 1>;
using Cell = prx::utilities::cube_cell_t<Eigen::MatrixXd, 2>;
using Grid = prx::utilities::regular_grid_t<Cell, 2>;

// FG
using SF = prx::fg::symbol_factory_t;
using FactorGraph = gtsam::NonlinearFactorGraph;
using Values = gtsam::Values;
using PropagationFactor = prx::fg::plant_propagation_CteStateCtrl_factor_t<State, Control, double>;
using NoiseModel = gtsam::noiseModel::Base::shared_ptr;
using DtPositiveFactor = prx::fg::constraint_factor_t<double, std::less<double>>;

template <typename T>
inline gtsam::Key state_key(const T t)
{
  return SF::create_hashed_symbol("X^{", t, "}");
}
template <typename T>
inline gtsam::Key control_key(const T t)
{
  return SF::create_hashed_symbol("U^{", t, "}");
}
template <typename T>
inline gtsam::Key dt_key(const T t)
{
  return SF::create_hashed_symbol("dt^{", t, "}");
}

template <typename Start, typename SystemGroup, typename Check, typename Grid>
std::set<std::size_t> propagate(const Start x0, SystemGroup sg, Check& check, prx::trajectory_t& traj, Grid& grid,
                                const std::string traj_file, std::size_t idx)
{
  std::set<std::size_t> res;
  traj.clear();
  check.reset();
  prx::space_t* ss{ sg->get_state_space() };
  prx::space_t* cs{ sg->get_control_space() };
  ss->copy_from(x0);
  traj.copy_onto_back(ss);

  // State xi;
  State xi{ Vec(traj.back()) };
  const Eigen::MatrixXd& K{ grid[idx]->element() };
  do
  {
    // ss->copy_to(x_curr);
    // const std::size_t idx{ grid.index(x_curr) };
    // const bool valid_k{ visited.count(idx) == 1 };
    // const Eigen::MatrixXd& K{ grid[idx]->element() };
    // PRX_DBG_VARS(idx, K, xi.transpose(), valid_k)
    // if (not valid_k)
    //   break;
    // if (valid_k)
    // {
    const Control u{ K * xi };
    cs->copy_from(u);
    // }

    // ctrl();
    sg->propagate_once();
    traj.copy_onto_back(ss);
    xi = Vec(traj.back());

    idx = grid.index(xi);
    res.insert(idx);
  } while (not check());
  traj.to_file(traj_file, std::ofstream::app);

  return res;
}

template <typename SystemGroup, typename Check, typename Grid>
std::set<std::size_t> check_lqr(const std::size_t& idx, SystemGroup sg, Check& check, prx::trajectory_t& traj,
                                Grid& grid, std::ofstream& vertices_ofs, const std::string traj_file)
{
  // const std::size_t idx{ grid.index(xT) };
  const std::vector<State> vertices{ grid[idx]->vertices() };
  std::set<std::size_t> ids;
  for (auto xi : vertices)
  {
    // vertices_ofs << xi.transpose() << "\n";
    vertices_ofs << xi.transpose() << std::endl;
    ids.merge(propagate(xi, sg, check, traj, grid, traj_file, idx));
  }
  return ids;
}

template <typename Vertices>
Eigen::MatrixXd lqr_to_neighbor(const std::string plant_name, const State& xT, const Vertices vertices)
{
  using PropagationCtrlFactor = prx::fg::plant_propagation_CteCteCtrl_factor_t<State, Control, double>;
  using PropagationStateCtrlFactor = prx::fg::plant_propagation_CteStateCtrl_factor_t<State, Control, double>;
  // using PropagationFactor = prx::fg::plant_propagation_CteStateCtrl_factor_t<State, Control, double>;

  Values values;
  FactorGraph graph, graph_to_eliminate;

  gtsam::Ordering ordering, odering_dts, ordering_xs, ordering_us;

  // const gtsam::Key xT_k{ state_key('T') };
  // const gtsam::Key uT_k{ control_key('T') };

  gtsam::noiseModel::Base::shared_ptr x_noise{ gtsam::noiseModel::Isotropic::Sigma(2, 1e-0) };
  gtsam::noiseModel::Base::shared_ptr dt_noise{ gtsam::noiseModel::Isotropic::Sigma(1, 1e-5) };

  const Control uT{ Control::Zero() };
  // values.insert(uT_k, uT);
  // graph.addPrior(uT_k, xT);
  // graph.addPrior(xT_k, xT);
  for (int i = 0; i < vertices.size(); ++i)
  {
    const State& xi{ vertices[i] };
    const gtsam::Key xi_k{ state_key(i) };
    const gtsam::Key uT_k{ control_key(i) };
    const gtsam::Key dti_k{ dt_key(i) };
    // graph.addPrior(xi_k, xi);
    // graph.addPrior(uT_k, uT);
    graph.emplace_shared<PropagationCtrlFactor>(xT, xi, uT_k, dti_k, x_noise, plant_name);
    graph.emplace_shared<DtPositiveFactor>(dti_k, prx::simulation_step, dt_noise);

    graph_to_eliminate.addPrior(xi_k, xi);
    graph_to_eliminate.emplace_shared<PropagationStateCtrlFactor>(xT, xi_k, uT_k, dti_k, x_noise, plant_name);

    auto K = Eigen::Vector2d(-0.874225, 0.00794384);
    Eigen::VectorXd ui = K.transpose() * xi;
    PRX_DBG_VARS(xi.transpose(), ui);
    values.insert(xi_k, xi);
    values.insert(dti_k, 0.5);
    values.insert(uT_k, ui);
    graph.addPrior(uT_k, ui);

    odering_dts.push_back(dti_k);
    ordering_xs.push_back(uT_k);
    ordering_xs.push_back(xi_k);

    break;
    // ordering_us.push_back(uT_k);
  }
  // ordering_xs.pop_back();
  ordering.insert(ordering.begin(), odering_dts.begin(), odering_dts.end());
  ordering.insert(ordering.end(), ordering_xs.begin(), ordering_xs.end() - 1);
  // ordering.insert(ordering.end(), ordering_us.begin(), ordering_us.end());
  // ordering.push_back(uT_k);
  // PRINT_KEYS(ordering);

  gtsam::LevenbergMarquardtParams lm_params{ prx::fg::default_levenberg_marquardt_parameters() };
  lm_params.setMaxIterations(10);

  gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
  gtsam::Values result{ optimizer.optimize() };

  result.print("result ", SF::formatter);
  graph.print("graph ", SF::formatter);
  graph_to_eliminate.print("graph_to_eliminate ", SF::formatter);
  PRX_DBG_VARS(graph.error(result), graph_to_eliminate.error(result));

  gtsam::GaussianFactorGraph linearized_graph{ *(graph_to_eliminate.linearize(result)) };

  Eigen::MatrixXd S, K, R;
  prx::fg::compute_K_S(linearized_graph, ordering, S, K, R);
  PRX_DBG_VARS(xT.transpose(), K)
  return K;
}

template <typename Vertices>
Eigen::MatrixXd bang_bang_lqr_to_neighbor(const std::string plant_name, const State& xT, const Vertices vertices)
{
  using PropagationFactor = prx::fg::plant_propagation_StateStateCtrl_factor_t<State, Control>;
  using PropagationCtrlFactor = prx::fg::plant_propagation_CteCteCtrl_factor_t<State, Control>;
  using PropagationStateCtrlFactor = prx::fg::plant_propagation_CteStateCtrl_factor_t<State, Control>;
  using PropagationStateCteFactor = prx::fg::plant_propagation_CteStateCte_factor_t<State, Control>;

  Values values;
  FactorGraph graph, graph_to_eliminate;

  gtsam::Ordering ordering, odering_dts, ordering_xs, ordering_us;

  gtsam::noiseModel::Base::shared_ptr x_noise{ gtsam::noiseModel::Isotropic::Sigma(2, 1e-0) };
  gtsam::noiseModel::Base::shared_ptr dt_noise{ gtsam::noiseModel::Isotropic::Sigma(1, 1e-5) };

  const gtsam::Key xT_k{ state_key('T') };
  const Control uT{ Control(0.6371781908344007) };
  // values.insert(uT_k, uT);
  // graph.addPrior(uT_k, xT);
  const double dt{ 0.5 };
  PropagationFactor pf(xT_k, state_key('i'), control_key('T'), nullptr, dt, plant_name);
  for (int i = 0; i < vertices.size(); ++i)
  {
    const State& xi{ vertices[i] };
    const gtsam::Key uT_k{ control_key(i) };
    const gtsam::Key xi_k{ state_key(i) };
    const gtsam::Key dti_k{ dt_key(i) };

    const State xTi{ pf.integrate(xi, uT, dt) };
    // graph.addPrior(xi_k, xi);
    // graph.addPrior(uT_k, uT);
    // graph.emplace_shared<PropagationCtrlFactor>(xT, xi, uT_k, 0.5, x_noise, plant_name);
    // graph.emplace_shared<DtPositiveFactor>(dti_k, prx::simulation_step, dt_noise);

    graph_to_eliminate.addPrior(xi_k, xi);
    graph_to_eliminate.emplace_shared<PropagationStateCtrlFactor>(xTi, xi_k, uT_k, dt, x_noise, plant_name);

    // auto K = Eigen::Vector2d(-0.874225, 0.00794384);
    // Eigen::VectorXd ui = K.transpose() * xi;
    PRX_DBG_VARS(xi.transpose(), xTi.transpose(), uT);
    values.insert(xi_k, xi);
    // values.insert(dti_k, 0.5);
    values.insert(uT_k, uT);
    // graph.addPrior(uT_k, ui);

    // odering_dts.push_back(dti_k);
    ordering_xs.push_back(uT_k);
    ordering_xs.push_back(xi_k);

    // break;
    // ordering_us.push_back(uT_k);
  }
  // ordering_xs.pop_back();
  // ordering.insert(ordering.begin(), odering_dts.begin(), odering_dts.end());
  ordering.insert(ordering.end(), ordering_xs.begin(), ordering_xs.end() - 1);
  // ordering.insert(ordering.end(), ordering_us.begin(), ordering_us.end());
  // ordering.push_back(uT_k);
  // PRINT_KEYS(ordering);

  gtsam::LevenbergMarquardtParams lm_params{ prx::fg::default_levenberg_marquardt_parameters() };
  lm_params.setMaxIterations(10);

  gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
  gtsam::Values result{ optimizer.optimize() };

  result.print("result ", SF::formatter);
  graph.print("graph ", SF::formatter);
  graph_to_eliminate.print("graph_to_eliminate ", SF::formatter);
  PRX_DBG_VARS(graph.error(result), graph_to_eliminate.error(result));

  gtsam::GaussianFactorGraph linearized_graph{ *(graph_to_eliminate.linearize(values)) };
  // gtsam::GaussianFactorGraph linearized_graph{ *(graph_to_eliminate.linearize(result)) };

  Eigen::MatrixXd S, K, R;
  prx::fg::compute_K_S(linearized_graph, ordering, S, K, R);
  PRX_DBG_VARS(xT.transpose(), K)
  return K;
}

Eigen::MatrixXd simple_lqr(const std::string plant_name, const State& xT, const State& x0)
{
  using PropagationCtrlFactor = prx::fg::plant_propagation_CteCteCtrl_factor_t<State, Control, double>;
  using PropagationStateCtrlFactor = prx::fg::plant_propagation_CteStateCtrl_factor_t<State, Control, double>;
  using PropagationStateCtrlFdtFactor = prx::fg::plant_propagation_CteStateCtrl_factor_t<State, Control>;
  using PropagationCteCtrlFactor = prx::fg::plant_propagation_CteCteCtrl_factor_t<State, Control, double>;
  using PropagationXXCtrlFdtFactor = prx::fg::plant_propagation_StateStateCtrl_factor_t<State, Control>;
  // using PropagationFactor = prx::fg::plant_propagation_CteStateCtrl_factor_t<State, Control, double>;

  Values values;
  FactorGraph graph, graph_to_eliminate;

  gtsam::Ordering ordering, odering_dts, ordering_xs, ordering_us;

  const gtsam::Key xT_k{ state_key('T') };
  // const gtsam::Key uT_k{ control_key('T') };

  gtsam::noiseModel::Base::shared_ptr x_noise{ gtsam::noiseModel::Isotropic::Sigma(2, 1e-0) };
  gtsam::noiseModel::Base::shared_ptr dt_noise{ gtsam::noiseModel::Isotropic::Sigma(1, 1e-5) };

  const Control uT{ Control::Zero() };
  // values.insert(uT_k, uT);
  // graph.addPrior(uT_k, xT);
  // graph.addPrior(xT_k, xT);
  // const State& xi{ vertices[i] };
  const gtsam::Key x0_k{ state_key(0) };
  const gtsam::Key uT_k{ control_key(0) };
  const gtsam::Key dti_k{ dt_key(0) };
  // graph.addPrior(xi_k, xi);
  // graph.addPrior(uT_k, uT);
  graph.emplace_shared<PropagationCtrlFactor>(xT, x0, uT_k, dti_k, x_noise, plant_name);
  graph.emplace_shared<DtPositiveFactor>(dti_k, prx::simulation_step, dt_noise);

  // graph_to_eliminate.addPrior(x0_k, x0);
  // graph_to_eliminate.emplace_shared<PropagationCteCtrlFactor>(xT, x0, uT_k, dti_k, x_noise, plant_name);
  // graph_to_eliminate.emplace_shared<PropagationStateCtrlFactor>(xT, x0_k, uT_k, dti_k, x_noise, plant_name);

  // values.insert(xi_k, xi);
  values.insert(dti_k, 0.5);
  values.insert(uT_k, uT);
  values.insert(x0_k, x0);
  values.insert(xT_k, xT);
  // graph.addPrior(uT_k, ui);

  // odering_dts.push_back(dti_k);
  // ordering_xs.push_back(uT_k);
  // ordering_xs.push_back(xi_k);
  // ordering.push_back(dti_k);
  ordering.push_back(uT_k);
  // ordering_xs.pop_back();
  // ordering.insert(ordering.begin(), odering_dts.begin(), odering_dts.end());
  // ordering.insert(ordering.end(), ordering_xs.begin(), ordering_xs.end() - 1);
  // ordering.insert(ordering.end(), ordering_us.begin(), ordering_us.end());
  // ordering.push_back(uT_k);
  // PRINT_KEYS(ordering);

  gtsam::LevenbergMarquardtParams lm_params{ prx::fg::default_levenberg_marquardt_parameters() };
  lm_params.setMaxIterations(10);

  gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
  gtsam::Values result{ optimizer.optimize() };

  result.print("result ", SF::formatter);
  graph.print("graph ", SF::formatter);
  graph_to_eliminate.print("graph_to_eliminate ", SF::formatter);
  PRX_DBG_VARS(graph.error(result), graph_to_eliminate.error(result));

  PropagationStateCtrlFactor pf(xT, x0_k, uT_k, dti_k, x_noise, plant_name);
  const double dt_res{ result.at<double>(dti_k) };
  const State xF{ pf.integrate(x0, result.at<Control>(uT_k), dt_res) };
  PRX_DBG_VARS(dt_res, xF.transpose());

  graph_to_eliminate.emplace_shared<PropagationXXCtrlFdtFactor>(xT_k, x0_k, uT_k, x_noise, dt_res, plant_name);
  gtsam::GaussianFactorGraph linearized_graph{ *(graph_to_eliminate.linearize(result)) };

  linearized_graph.print("linearized FG ", SF::formatter);
  Eigen::MatrixXd S, K, R;
  prx::fg::compute_K_S(linearized_graph, ordering, S, K, R);
  linearized_graph.print("linearized FG ", SF::formatter);
  PRX_DBG_VARS(xT.transpose(), K)
  PRX_DBG_VARS(S, R)
  return K;
}

int main(int argc, char* argv[])
{
  const double& pi{ prx::constants::pi };

  prx::param_loader params{};
  params["traj_file"].set(prx::out_path + "/traj_cell_dbg.txt");
  params["vertices_file"].set(prx::out_path + "/vertices_cell_dbg.txt");
  params.add_opts(argc, argv);

  const std::string TRAJ_FILE{ params["traj_file"].as<>() };
  const std::string VERTICES_FILE{ params["vertices_file"].as<>() };
  std::ofstream vertices_ofs(VERTICES_FILE.c_str());

  prx::simulation_step = 0.01;
  const std::string plant_name{ "pendulum" };
  auto plant = prx::system_factory_t::create_system(plant_name, plant_name);
  prx_assert(plant != nullptr, "Plant is nullptr!");

  prx::world_model_t world_model({ plant }, {});
  world_model.create_context("context", { plant_name }, {});
  auto context = world_model.get_context("context");
  std::shared_ptr<prx::system_group_t> sg{ prx::system_group(context) };

  prx::space_t* ss{ sg->get_state_space() };
  prx::space_t* cs{ sg->get_control_space() };
  prx::space_t* ps{ sg->get_parameter_space() };

  prx::trajectory_t traj(ss);
  traj.to_file(TRAJ_FILE);

  const double length_th{ 0.1 };
  const double length_dth{ 0.1 };
  const Cell::Coordinate min(-pi, -2 * pi);
  const Cell::Coordinate max(+pi, +2 * pi);
  const Cell::Coordinate cell_length(length_th, length_dth);
  Grid grid(min, max, cell_length, Eigen::Matrix<double, 1, 2>::Zero());

  State x0{ State::Zero() };
  State xT{ State::Zero() };

  Eigen::MatrixXd K;

  // std::set<std::size_t> visited;
  // std::queue<std::size_t> to_visit;
  // std::map<std::size_t, std::queue<std::size_t>> neighbors_to_visit;

  // to_visit.push(xT_idx);
  // neighbors_to_visit.emplace(xT_idx);
  // neighbors_to_visit[xT_idx].push(xT_idx);

  // bool valid_k;
  State x_curr{ State::Zero() };

  prx::condition_check_t cond_check("sim_time", 5);
  // std::function<void()> check = [&]() { return valid_k and cond_check(); };
  int cont;

  ss->copy(x0, params["x0"].as<std::vector<double>>());
  ss->copy(xT, params["xT"].as<std::vector<double>>());

  std::size_t x0_idx{ grid.index(x0) };
  auto cellptr = grid[x0_idx];
  K = simple_lqr(plant_name, xT, x0);
  // K = lqr_to_neighbor(plant_name, xT, cellptr->vertices());
  // K = bang_bang_lqr_to_neighbor(plant_name, xT, cellptr->vertices());
  cellptr->element() = K;
  // visited.insert(idx);
  std::set<std::size_t> reached_ids{ check_lqr(x0_idx, sg, cond_check, traj, grid, vertices_ofs, TRAJ_FILE) };

  vertices_ofs.close();
  // PRX_DBG_VARS(idx, xT.transpose());
  // PRX_DBG_VARS(reached_ids);

  // for (auto next_idx : reached_ids)
  // {
  //   if (visited.count(next_idx) == 0)
  //   {
  //     to_visit.push(next_idx);
  //     neighbors_to_visit[next_idx].push(idx);
  //   }
  // }

  // PRX_DBG_VARS(to_visit);
  // std::cin >> cont;

  return 0;
}
