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

using prx::utilities::convert_to;
using SF = prx::fg::symbol_factory_t;
using CsvReader = prx::utilities::csv_reader_t;
using State = Eigen::Vector2d;
using Control = Eigen::Vector<double, 1>;
using FactorGraph = gtsam::NonlinearFactorGraph;
using Values = gtsam::Values;
using PropagationFactor = prx::fg::plant_propagation_CteStateCtrl_factor_t<State, Control, double>;
using NoiseModel = gtsam::noiseModel::Base::shared_ptr;
using DtPositiveFactor = prx::fg::constraint_factor_t<double, std::less<double>>;

const std::string TRAJ_FILE{ prx::out_path + "/traj_dbg.txt" };
const std::string VERTICES_FILE{ prx::out_path + "/vertices_dbg.txt" };
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

template <typename Start, typename SystemGroup, typename Ctrl, typename Check, typename Grid>
std::set<std::size_t> propagate(const Start x0, SystemGroup sg, Ctrl& ctrl, Check& check, prx::trajectory_t& traj,
                                Grid& grid)
{
  std::set<std::size_t> res;
  traj.clear();
  check.reset();
  prx::space_t* ss{ sg->get_state_space() };
  ss->copy_from(x0);
  traj.copy_onto_back(ss);
  do
  {
    ctrl();
    sg->propagate_once(nullptr);
    traj.copy_onto_back(ss);
    res.insert(grid.index(Vec(traj.back())));
  } while (not check());
  traj.to_file(TRAJ_FILE, std::ofstream::app);

  return res;
}

template <typename SystemGroup, typename Ctrl, typename Check, typename Grid>
std::set<std::size_t> check_lqr(const State& xT, SystemGroup sg, Ctrl& ctrl, Check& check, prx::trajectory_t& traj,
                                Grid& grid, std::ofstream& vertices_ofs)
{
  const std::vector<State> vertices{ grid(xT)->vertices() };
  std::set<std::size_t> ids;
  for (auto xi : vertices)
  {
    vertices_ofs << xi.transpose() << "\n";
    ids.merge(propagate(xi, sg, ctrl, check, traj, grid));
  }
  return ids;
}

template <typename Vertices>
Eigen::MatrixXd lqr_to_neighbor(const std::string plant_name, const State& xT, const Vertices vertices)
{
  using PropagationCtrlFactor = prx::fg::plant_propagation_CteCteCtrl_factor_t<State, Control, double>;
  using PropagationStateCtrlFactor = prx::fg::plant_propagation_CteStateCtrl_factor_t<State, Control, double>;
  // using PropagationFactor = prx::fg::plant_propagation_CteStateCtrl_factor_t<State, Control, double>;

  gtsam::Values values;
  gtsam::NonlinearFactorGraph graph, graph_to_eliminate;

  gtsam::Ordering ordering, odering_dts, ordering_xs;

  // const gtsam::Key xT_k{ state_key('T') };
  const gtsam::Key uT_k{ control_key('T') };

  gtsam::noiseModel::Base::shared_ptr x_noise{ gtsam::noiseModel::Isotropic::Sigma(2, 1e-0) };
  gtsam::noiseModel::Base::shared_ptr dt_noise{ gtsam::noiseModel::Isotropic::Sigma(1, 1e-5) };

  const Control uT{ Control::Zero() };
  values.insert(uT_k, uT);
  // graph.addPrior(uT_k, xT);
  // graph.addPrior(xT_k, xT);
  for (int i = 0; i < vertices.size(); ++i)
  {
    const State& xi{ vertices[i] };
    const gtsam::Key xi_k{ state_key(i) };
    const gtsam::Key dti_k{ dt_key(i) };
    // graph.addPrior(xi_k, xi);
    // graph.addPrior(uT_k, uT);
    graph.emplace_shared<PropagationCtrlFactor>(xT, xi, uT_k, dti_k, x_noise, plant_name);
    graph.emplace_shared<DtPositiveFactor>(dti_k, prx::simulation_step, dt_noise);

    graph_to_eliminate.addPrior(xi_k, xi);
    graph_to_eliminate.emplace_shared<PropagationStateCtrlFactor>(xT, xi_k, uT_k, dti_k, x_noise, plant_name);

    values.insert(xi_k, xi);
    values.insert(dti_k, 0.1);

    odering_dts.push_back(dti_k);
    ordering_xs.push_back(xi_k);
  }
  ordering.insert(ordering.begin(), odering_dts.begin(), odering_dts.end());
  ordering.insert(ordering.end(), ordering_xs.begin(), ordering_xs.end() - 1);
  ordering.push_back(uT_k);
  // ordering.push_back(ordering_xs.back());

  // PRX_DBG_VARS(ordering);

  gtsam::LevenbergMarquardtParams lm_params{ prx::fg::default_levenberg_marquardt_parameters() };
  lm_params.setMaxIterations(10);

  gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
  gtsam::Values result{ optimizer.optimize() };
  // graph_to_eliminate.addPrior(uT_k, result.at<Control>(uT_k));

  // result.print("Result", SF::formatter);
  PRX_DBG_VARS(graph.error(result), graph_to_eliminate.error(result));
  // graph_to_eliminate.printErrors(result, "Result", SF::formatter);

  gtsam::GaussianFactorGraph linearized_graph{ *(graph_to_eliminate.linearize(result)) };

  // linearized_graph.print("Linear Graph", SF::formatter);
  // PRINT_KEYS(ordering);
  Eigen::MatrixXd S, K, R;
  // for (auto ki : ordering)
  // {
  //   gtsam::Ordering lo;
  //   lo.push_back(ki);
  //   PRINT_KEYS(lo);
  prx::fg::compute_K_S(linearized_graph, ordering, S, K, R);
  //   linearized_graph.print("Linear Graph", SF::formatter);
  // }
  return K;
}

int main(int argc, char* argv[])
{
  prx::param_loader params{};
  params.add_opts(argc, argv);

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

  std::ofstream vertices_ofs(VERTICES_FILE.c_str());

  Values values;
  FactorGraph graph;

  gtsam::noiseModel::Base::shared_ptr x_noise{ gtsam::noiseModel::Isotropic::Sigma(2, 1e-0) };
  gtsam::noiseModel::Base::shared_ptr dt_noise{ gtsam::noiseModel::Isotropic::Sigma(1, 1e-5) };

  const gtsam::Key xT_k{ state_key('T') };
  const gtsam::Key u0_k{ control_key(0) };
  const gtsam::Key uT_k{ control_key('T') };
  const gtsam::Key dt_k{ dt_key('T') };

  State xT{ State::Zero() };
  const Control uT{ Control::Zero() };

  gtsam::NonlinearFactorGraph graph_T;
  graph.addPrior(xT_k, xT);
  graph_T.emplace_shared<PropagationFactor>(xT, xT_k, uT_k, dt_k, x_noise, plant_name);
  graph_T.emplace_shared<DtPositiveFactor>(dt_k, prx::simulation_step, dt_noise);
  graph_T.addPrior(uT_k, uT);
  values.insert(uT_k, uT);
  // values.insert(u0_k, uT);
  values.insert(xT_k, xT);
  values.insert(dt_k, 0.1);

  const double delta{ 0.1 };
  const std::vector<State> xs{ { State(+delta, +delta), State(+delta, -delta), State(-delta, -delta),
                                 State(-delta, +delta) } };
  // for (int i = 1; i <= 4; ++i)
  // {
  //   const gtsam::Key xi_key{ state_key(i - 1) };
  //   const gtsam::Key dti_k{ dt_key(i - 1) };
  //   const State xi{ xT + xs[i - 1] };
  //   PRX_DBG_VARS(i, xi.transpose());
  //   // graph.addPrior(xi_key, xi);
  //   const gtsam::Key ui_k{ control_key(i - 1) };
  //   values.insert(ui_k, uT);
  //   values.insert(dti_k, 0.1);
  //   values.insert(xi_key, xi);
  //   graph.emplace_shared<PropagationFactor>(xT, xi_key, ui_k, dti_k, x_noise, plant_name);
  //   graph.emplace_shared<DtPositiveFactor>(dti_k, prx::simulation_step, dt_noise);
  //   // values.insert(xi_key, xi);
  // }

  // graph += graph_T;
  gtsam::LevenbergMarquardtParams lm_params{ prx::fg::default_levenberg_marquardt_parameters() };
  lm_params.setMaxIterations(100);

  gtsam::LevenbergMarquardtOptimizer optimizer(graph_T, values, lm_params);
  gtsam::Values result{ optimizer.optimize() };

  result.print("Values", SF::formatter);
  graph_T.printErrors(result, "Result", SF::formatter);

  gtsam::Ordering odering;

  odering.push_back(dt_k);
  odering.push_back(uT_k);
  // odering.push_back(xT_k);
  gtsam::GaussianFactorGraph linearized_graph{ *(graph_T.linearize(result)) };

  linearized_graph.print("Linear Graph", SF::formatter);
  Eigen::MatrixXd S, K, R;
  prx::fg::compute_K_S(linearized_graph, odering, S, K, R);
  linearized_graph.print("Linear Graph", SF::formatter);
  PRX_DBG_VARS(K);

  State x_curr{};
  Control u_test{};

  using Cell = prx::utilities::cube_cell_t<Eigen::MatrixXd, 2>;
  using Grid = prx::utilities::regular_grid_t<Cell, 2>;

  const double& pi{ prx::constants::pi };
  const Cell::Coordinate min(-pi, -2 * pi);
  const Cell::Coordinate max(+pi, +2 * pi);
  const Cell::Coordinate cell_length(0.1, 0.1);
  Grid grid(min, max, cell_length, Eigen::Matrix<double, 1, 2>::Zero());

  // K = lqr_to_neighbor(plant_name, xT, grid(xT)->vertices());
  // PRX_DBG_VARS(K);

  // grid(xT)->element() = K;
  std::function<void()> ctrl = [&]() {
    ss->copy_to(x_curr);
    const Eigen::MatrixXd& K{ grid(x_curr)->element() };

    // const Control u{ -K * x_curr };
    const Control u{ K * x_curr };
    // PRX_DBG_VARS(x_curr.transpose(), K, u);
    cs->copy_from(u);
  };

  // const std::vector<State> vertices{ grid(xT)->vertices() };

  prx::condition_check_t cond_check("sim_time", 0.5);
  State xR;
  // prx::trajectory_t traj(ss);
  // traj.to_file(TRAJ_FILE);
  // std::ios_base::openmode mode{ std::ofstream::trunc };
  // check_lqr(vertices, sg, ctrl, cond_check);
  // for (auto vi : vertices)
  // {
  //   PRX_DBG_VARS(vi.transpose());
  //   ss->copy_from(vi);
  //   traj.copy_onto_back(ss);
  //   do
  //   {
  //     ctrl();
  //     sg->propagate_once(nullptr);
  //     traj.copy_onto_back(ss);
  //   } while (not cond_check());
  //   // sg->propagate(vi, ctrl, cond_check, traj);
  //   cond_check.reset();
  // traj.to_file(TRAJ_FILE, mode);
  //   traj.clear();
  //   mode = std::ofstream::app;
  // }

  std::set<std::size_t> visited;
  std::set<std::size_t> to_visit;
  // visited.insert(grid.index(xT));

  // to_visit.insert(grid.index(xT));
  // while (to_visit.size() > 0)
  // {
  //   xT = grid[*(to_visit.begin())]->vertex(0);
  //   to_visit.erase(to_visit.begin());
  //   std::set<std::size_t> reached_ids{ check_lqr(xT, sg, ctrl, cond_check, traj, grid, vertices_ofs) };
  //   // to_visit.insert();
  //   PRX_DBG_VARS(visited);
  //   PRX_DBG_VARS(reached_ids);
  //   std::set_difference(reached_ids.begin(), reached_ids.end(),  // no-lint
  //                       visited.begin(), visited.end(),          // no-lint
  //                       std::inserter(to_visit, to_visit.end()));
  //   PRX_DBG_VARS(to_visit.size(), to_visit);
  // }
  return 0;
  // State xR{ State::Zero() };
  // for (auto xi : vertices)
  // {
  //   // const State xi{ xT + xs[i - 1] };
  //   sg->propagate(xi, ctrl, check, xR);
  //   PRX_DBG_VARS(xi.transpose(), xR.transpose());
  //   // break;
  // }

  // std::size_t idx_curr, idx_desire;
  // State x_check;
  // std::function<bool()> check = [&]() {
  //   ss->copy_to(x_check);
  //   const std::size_t idx{ grid.index(x_check) };
  //   PRX_DBG_VARS(idx, idx_curr, idx_desire, x_check.transpose());
  //   if (idx == idx_curr)
  //   {
  //     return false;
  //   }
  //   if (idx == idx_desire)
  //   {
  //     return true;
  //   }
  //   return true;
  // };

  // const std::vector<std::size_t> neighbors{ grid.neighbors(xT) };
  // for (auto ni : neighbors)
  // {
  //   PRX_DBG_VARS(grid[ni]->vertex(0).transpose());
  //   const std::vector<State> vertices{ grid[ni]->vertices() };
  //   const Eigen::MatrixXd ki{ lqr_to_neighbor(plant_name, xT, vertices) };

  //   idx_curr = ni;
  //   idx_desire = grid.index(xT);
  //   cond_check.reset();
  //   // check_lqr(vertices, sg, ctrl, check);
  //   PRX_DBG_VARS(grid.index(xR), idx_desire, xR.transpose());
  // }

  // return 0;
};
