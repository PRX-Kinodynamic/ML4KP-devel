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
using Controller = std::function<Control(const State&)>;
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

template <typename SystemGroup, typename Check, typename Grid>
std::set<std::size_t> check_lqr(const std::size_t& idx, SystemGroup sg, Check& check, prx::trajectory_t& traj,
                                Grid& grid, std::ofstream& vertices_ofs, const std::string traj_file,
                                std::set<std::size_t>& visited)
{
  // const std::size_t idx{ grid.index(xT) };
  const std::vector<State> vertices{ grid[idx]->vertices() };
  std::set<std::size_t> ids;
  for (auto xi : vertices)
  {
    // vertices_ofs << xi.transpose() << "\n";
    vertices_ofs << xi.transpose() << std::endl;
    // ids.merge(propagate(xi, sg, check, traj, grid, traj_file, idx, visited));
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
    // const gtsam::Key uT_k{ control_key(i) };
    const gtsam::Key dti_k{ dt_key(i) };
    // graph.addPrior(xi_k, xi);
    // graph.addPrior(uT_k, uT);
    graph.emplace_shared<PropagationCtrlFactor>(xT, xi, uT_k, dti_k, x_noise, plant_name);
    graph.emplace_shared<DtPositiveFactor>(dti_k, prx::simulation_step, dt_noise);

    graph_to_eliminate.addPrior(xi_k, xi);
    graph_to_eliminate.emplace_shared<PropagationStateCtrlFactor>(xT, xi_k, uT_k, dti_k, x_noise, plant_name);

    values.insert(xi_k, xi);
    values.insert(dti_k, 0.5);
    // values.insert(uT_k, uT);

    odering_dts.push_back(dti_k);
    ordering_xs.push_back(xi_k);
    // ordering_us.push_back(uT_k);
  }
  ordering.insert(ordering.begin(), odering_dts.begin(), odering_dts.end());
  ordering.insert(ordering.end(), ordering_xs.begin(), ordering_xs.end() - 1);
  // ordering.insert(ordering.end(), ordering_us.begin(), ordering_us.end());
  ordering.push_back(uT_k);

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

/////// NEW FUNCTIONS /////
Controller lqr_controller(const State& xt, const State& local_goal)
{
  return [](const State& x) -> Control {
    const Eigen::Matrix<double, 1, 2> K{ 7.39050619, 2.60611851 };
    return -K * x;
  };
}

template <typename SystemGroup, typename Check, typename Grid>
std::size_t propagate(const State& x0, SystemGroup sg, Check& check, prx::trajectory_t& traj, Grid& grid,
                      const std::string traj_file, std::size_t x0_idx, Controller& u)
{
  std::size_t res;
  traj.clear();
  check.reset();
  prx::space_t* ss{ sg->get_state_space() };
  prx::space_t* cs{ sg->get_control_space() };
  ss->copy_from(x0);
  traj.copy_onto_back(ss);

  std::size_t idx{ 0 };
  State xi{ Vec(traj.back()) };
  do
  {
    const Control ut{ u(xi) };
    cs->copy_from(ut);
    // PRX_DBG_VARS(ut.transpose());
    sg->propagate_once();
    traj.copy_onto_back(ss);
    xi = Vec(traj.back());

    idx = grid.index(xi);
  } while (idx == x0_idx and not check());
  // } while (not check());
  traj.to_file(traj_file, std::ofstream::app);

  return idx;
}

template <typename Grid, typename SystemGroup, typename Check>
std::vector<std::size_t> fwd_prop(Grid& grid, Controller& u, const std::size_t idx, SystemGroup sg, Check& check,
                                  prx::trajectory_t& traj, const std::string traj_file, std::ofstream& vertices_ofs)
{
  const std::vector<State> vertices{ grid[idx]->vertices() };

  std::vector<std::size_t> ids;
  for (auto xi : vertices)
  {
    vertices_ofs << xi.transpose() << "\n";
    ids.push_back(propagate(xi, sg, check, traj, grid, traj_file, idx, u));
  }
  return ids;
}

struct morse_graph_t : public std::enable_shared_from_this<morse_graph_t>
{
  using MGptr = std::shared_ptr<morse_graph_t>;

  morse_graph_t() : idx(0), visited(false) {};
  morse_graph_t(const std::size_t i, const bool explored) : idx(i), visited(explored) {};

  static std::shared_ptr<morse_graph_t> create(const std::size_t i, const bool explored)
  {
    return std::make_shared<morse_graph_t>(i, explored);
  }

  friend std::ostream& operator<<(std::ostream& os, const morse_graph_t& obj)
  {
    os << "idx: " << obj.idx << " visited: " << obj.visited << "\n";
    os << "\ttargets: [ ";
    for (auto t : obj.targets)
    {
      os << t->idx << " ";
    }
    os << "]\n\tsources: [ ";
    for (auto s : obj.sources)
    {
      os << s->idx << " ";
    }
    os << "]\n";
    const std::size_t tot_c{ obj.candidates.size() };
    os << "\t Candidates: " << tot_c << " ";
    if (tot_c > 0)
    {
      os << "(top): " << obj.candidates.front().transpose() << " ";
    }

    return os;
  }

  friend std::ostream& operator<<(std::ostream& os, const MGptr& obj)
  {
    os << *obj;
    return os;
  }

  State state;  // Representative of this node, mostly for dbg
  std::size_t idx;
  bool visited;

  std::queue<State> candidates;  // Other graph this one points to
  std::vector<MGptr> targets;    // Other graph this one points to
  std::vector<MGptr> sources;    // Only used for efficiency, not part of the Graph
};

template <typename Graph, typename IdsContainer>
std::vector<std::size_t> update_graph(Graph& graph, const std::size_t& idx_curr, IdsContainer& ids)
{
  morse_graph_t::MGptr curr{ graph[idx_curr] };

  std::vector<std::size_t> to_visit;
  prx_assert(curr != nullptr, "idx_curr: " << idx_curr << " has not been added to graph.");
  for (auto id : ids)
  {
    morse_graph_t::MGptr next{ nullptr };
    if (graph.count(id) == 0)  // Id is not in the graph, cube has not been visited
    {
      next = morse_graph_t::create(id, false);
      graph[id] = next;
      to_visit.push_back(id);
      // PRX_DBG_VARS("New ", id);
    }
    else
    {
      next = graph[id];
      // PRX_DBG_VARS("Existing ", id);
    }
    curr->targets.push_back(next);
  }
  curr->visited = true;
  PRX_DBG_VARS(curr);
  PRX_DBG_VARS(to_visit);

  return to_visit;
}

template <typename Graph>
void graph_to_file(const Graph& graph, const std::string filename)
{
  std::ofstream graph_ofs(filename.c_str());
  for (auto e : graph)
  {
    // graph
  }
}

int main(int argc, char* argv[])
{
  const double& pi{ prx::constants::pi };

  prx::param_loader params{};
  params["traj_file"].set(prx::out_path + "/traj_dbg.txt");
  params["vertices_file"].set(prx::out_path + "/vertices_dbg.txt");
  params["graph_file"].set(prx::out_path + "/graph_dbg.txt");
  params.add_opts(argc, argv);

  const std::string TRAJ_FILE{ params["traj_file"].as<>() };
  const std::string VERTICES_FILE{ params["vertices_file"].as<>() };
  const std::string GRAPH_FILE{ params["graph_file"].as<>() };
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

  State xT{ State::Zero() };
  std::size_t xT_idx{ grid.index(xT) };

  Eigen::MatrixXd K;

  std::set<std::size_t> visited;
  std::queue<State> to_visit;
  // std::map<std::size_t, std::queue<State>> neighbors_to_visit;
  std::map<std::size_t, morse_graph_t::MGptr> graph;

  to_visit.push(xT);
  graph[xT_idx] = morse_graph_t::create(xT_idx, false);
  graph[xT_idx]->candidates.push(xT);
  // neighbors_to_visit[xT_idx].push(xT);

  State x_curr{ State::Zero() };

  prx::condition_check_t cond_check("sim_time", 0.5);

  int cont;
  while (to_visit.size() > 0)
  {
    const State xt{ to_visit.front() };
    const std::size_t idx_xt{ grid.index(xt) };

    if (graph.count(idx_xt) > 0 and graph[idx_xt]->visited)
    {
      const std::string msg{ "Cube has been visited, skipping: " };
      PRX_DBG_VARS(msg, idx_xt);
      continue;
    }

    PRX_DBG_VARS(idx_xt, xt.transpose());
    to_visit.pop();

    PRX_DBG_VARS(graph.count(idx_xt));
    PRX_DBG_VARS(graph[idx_xt]);

    State candidate{ graph[idx_xt]->candidates.front() };
    graph[idx_xt]->candidates.pop();

    PRX_DBG_VARS(candidate.transpose());

    auto u = lqr_controller(xt, candidate);

    // auto xaux = grid[idx_xt]->vertex(0);
    // PRX_DBG_VARS(xaux.transpose(), u(xaux));

    // PRX_DBG_VARS(idx_xt, xt.transpose());
    // auto cube = grid[idx_xt];
    std::vector<std::size_t> reached_ids{ fwd_prop(grid, u, idx_xt, sg, cond_check, traj, TRAJ_FILE, vertices_ofs) };

    std::vector<std::size_t> next{ update_graph(graph, idx_xt, reached_ids) };

    for (auto i : next)
    {
      const State ci{ grid[i]->center() };
      to_visit.push(ci);
      graph[i]->state = ci;
      graph[i]->candidates.push(xt);
    }

    vertices_ofs.flush();
    std::cin >> cont;  // To iterate w/terminal
  }

  vertices_ofs.close();
  return 0;
}