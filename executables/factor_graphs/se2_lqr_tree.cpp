#include <fstream>
#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/aorrt.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/planners/planner.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/planning/planner_functions/tree_fix_time_discretization.hpp"
#include "prx/factor_graphs/plants/SE2_rigid_body.hpp"
#include "prx/factor_graphs/utilities/default_parameters.hpp"
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/linear/GaussianBayesNet.h>

using SF = prx::fg::symbol_factory_t;
using Integrator = prx::fg::lie_integration_factor_t<prx::fg::SE2_t, Eigen::Vector3d>;
// Compute the S on riccati / cost-to-go (x' S x)
template <typename Graph>
Eigen::Matrix<double, 3, 3> compute_S(Graph& graph, const gtsam::Key key)
{
  // '''Returns the value function matrix at variable `key` given a graph which
  //     goes up and including `key`, but no further (i.e. all time steps after
  //     `key` have already been eliminated).  Does so by aggregating all unary
  //     factors on `key`.  If value function is x^TPx, then this returns P.
  //     "Return Cost" aka "Cost-to-go" aka "Value Function".
  // Arguments:
  //     graph: factor graph in LTI form
  //     key: key in the factor graph for which we want to obtain the return cost
  // Returns:
  //     return_cost: return cost, an nxn array where `n` is dimension of `key`
  // '''
  gtsam::GaussianFactorGraph new_fg{};
  // graph->print("Original FG (S)", SF::formatter);
  for (std::size_t i = 0; i < graph->size(); ++i)
  {
    auto f = graph->at(i);
    if (f->keys().size() == 1 and f->keys()[0] == key)  // # collect unary factors on `key`
    {
      new_fg.push_back(f);
    }
  }
  // new_fg.print("New FG (S)", SF::formatter);
  auto sol_end = new_fg.eliminateSequential();
  auto S = sol_end->back()->information();

  // PRX_DBG_VARS("Ricatti:", S);
  return S;
}

template <typename Tree>
void tree_to_file(const std::string& filename, Tree& tree)
{
  std::ofstream ofs_tree(filename);
  ofs_tree << "# Source Target";

  auto edges_iters = tree->edges();
  for (auto iter = edges_iters.first; iter != edges_iters.second; iter++)
  {
    const std::shared_ptr<prx::aorrt_edge_t> edge{ std::dynamic_pointer_cast<prx::aorrt_edge_t>(*iter) };
    const std::size_t parent_id{ edge->get_source() };
    const std::size_t target_id{ edge->get_target() };

    const gtsam::Key key_xt0{ SF::create_hashed_symbol("X^{", parent_id, "}") };
    const gtsam::Key key_xt1{ SF::create_hashed_symbol("X^{", target_id, "}") };
    //
    // const gtsam::Key key_xdot{ SF::create_hashed_symbol("U^{", parent_id, "}_{", target_id, "}") };
    ofs_tree << SF::formatter(key_xt0) << " " << SF::formatter(key_xt1) << "\n";
  }
  ofs_tree.close();
}

template <typename Graph, typename Xkeys, typename Ukeys, typename Sout, typename Kout>
void compute_K_S(Graph& graph, const Xkeys& X, const Ukeys& U, Sout& Ss, Kout& Ks)
{
  // def get_k_and_p(graph, X, U):
  // '''Finds optimal control law given by $u=Kx$ and value function $Vx^2$ aka
  //     cost-to-go which corresponds to solutions to the algebraic, finite
  //     horizon Ricatti Equation.  K is Extracted from the bayes net and V is
  //     extracted by incrementally eliminating the factor graph.  If you only
  //     need K and not V, then use the `get_k` function below.
  // Arguments:
  //     graph: factor graph containing factor graph in LQR form
  //     X: list of state Keys
  //     U: list of control Keys
  // Returns:
  //     K: optimal control matrix, shape (T-1, 1)
  //     V: value function, shape (T, 1)
  //         TODO(gerry): support n-dimensional state space
  // '''
  // # Find K and V by using bayes net solution
  auto marginalized_fg = graph;

  // PRX_DBG_VARS(X.size(), U.size());
  Ss[X.back()] = compute_S(marginalized_fg, X.back());
  // for i in range(len(U)-2, -1, -1): # traverse backwards in time
  for (int i = U.size() - 1; i > -1; --i)  // # traverse backwards in time
  {
    // PRX_DBG_VARS(i);
    const gtsam::Key keyX1{ X[i + 1] };
    const gtsam::Key keyX0{ X[i] };
    const gtsam::Key keyU01{ U[i] };

    const std::string strKeyX1{ SF::formatter(X[i + 1]) };
    const std::string strKeyX0{ SF::formatter(X[i]) };
    const std::string strKeyU01{ SF::formatter(U[i]) };

    // PRX_DBG_VARS(strKeyX1, strKeyX0, strKeyU01);
    gtsam::Ordering ordering{};
    ordering.push_back(keyX1);
    ordering.push_back(keyU01);

    // std::pair<boost::shared_ptr<BayesNetType>, boost::shared_ptr<FactorGraphType> >
    // bayes_net, marginalized_fg = marginalized_fg.eliminatePartialSequential(ordering);
    auto pair_eliminated = marginalized_fg->eliminatePartialSequential(ordering);
    marginalized_fg = pair_eliminated.second;
    Ss[keyX0] = compute_S(marginalized_fg, keyX0);
    // K = solve(A, b)
    // K[i] = solve_triangular(pair_eliminated.first->back().R(), pair_eliminated.first->back().S())

    // pair_eliminated.first->print("BayesNet", SF::formatter);
    // K =
    const Eigen::MatrixXd R{ pair_eliminated.first->back()->R().matrix() };
    const Eigen::MatrixXd S{ pair_eliminated.first->back()->S().matrix() };
    // PRX_DBG_VARS(R);
    // PRX_DBG_VARS(S);

    Ks[keyX1] = R.triangularView<Eigen::Upper>().solve(S);
  }
}

int main(int argc, char* argv[])
{
  prx::param_loader params{};
  params["simulation_step"].set(0.1);
  params["random_seed"].set(123);
  params["environment"].set("environments/simple_obstacle.yaml");
  params["checker_type"].set("time");
  params["checker_value"].set(60);
  params["state_space/min_bound"].set(std::vector<double>({ -5, -5, -3.14159 }));
  params["state_space/max_bound"].set(std::vector<double>({ 25, 25, 3.14159 }));
  params["control_space/min_bound"].set(std::vector<double>({ -0.5, -0.5, -0.1 }));
  params["control_space/max_bound"].set(std::vector<double>({ 0.5, 0.5, 0.1 }));
  params["query/goal/state"].set(std::vector<double>({ 20.0, 20.0, 0 }));
  params["query/goal/radius"].set(0.2);
  params["query/total_solutions"].set(2);
  params["query/visualize"].set(true);
  params["query/start_state"].set(std::vector<double>({ 0.0, 0.0, 0.0 }));
  params["FG/LM"] = prx::fg::levenberg_marquardt::default_params();
  params["out/Kfile"].set(prx::out_path + "/se2_kfile.txt");
  params["out/Sfile"].set(prx::out_path + "/se2_sfile.txt");
  params["out/tree"].set(prx::out_path + "/lqr_tree.txt");

  params.add_opts(argc, argv);
  params.print();

  prx::simulation_step = params["simulation_step"].as<double>();
  prx::init_random(params["random_seed"].as<int>());

  auto obstacles = prx::load_obstacles(params["environment"].as<>());
  std::vector<std::shared_ptr<prx::movable_object_t>> obstacle_list = obstacles.second;
  std::vector<std::string> obstacle_names = obstacles.first;

  const std::string plant_name{ "SE2_rigid_body_1st_order" };
  auto plant = prx::system_factory_t::create_system(plant_name, plant_name);
  prx_assert(plant != nullptr, "Plant is nullptr!");

  prx::world_model_t world_model({ plant }, { obstacle_list });
  world_model.create_context("context", { plant_name }, { obstacle_names });
  auto context = world_model.get_context("context");

  std::shared_ptr<prx::system_group_t> sg{ prx::system_group(context) };
  std::shared_ptr<prx::collision_group_t> cg{ prx::collision_group(context) };

  prx::space_t* ss{ sg->get_state_space() };
  prx::space_t* cs{ sg->get_control_space() };

  prx::aorrt_t aorrt("aorrt");
  prx::aorrt_specification_t aorrt_spec(context.first, context.second);
  aorrt_spec.bnb = true;

  const std::vector<double> ss_min_bound{ params["state_space/min_bound"].as<std::vector<double>>() };
  const std::vector<double> ss_max_bound{ params["state_space/max_bound"].as<std::vector<double>>() };

  const std::vector<double> cs_min_bound{ params["control_space/min_bound"].as<std::vector<double>>() };
  const std::vector<double> cs_max_bound{ params["control_space/max_bound"].as<std::vector<double>>() };

  ss->set_bounds(ss_min_bound, ss_max_bound);
  cs->set_bounds(cs_min_bound, cs_max_bound);

  prx::aorrt_query_t aorrt_query(context.first->get_state_space(), context.first->get_control_space());
  aorrt_query.init(params["query"]);
  // aorrt_query.start_state = ss->make_point(params["start_state"].as<std::vector<double>>());
  // aorrt_query.goal_state = ss->make_point(params["goal/state"].as<std::vector<double>>());
  // aorrt_query.goal_region_radius = params["goal/radius"].as<double>();
  // aorrt_query.get_visualization = params["visualize"].as<bool>();

  aorrt.link_and_setup_spec(&aorrt_spec);
  aorrt.preprocess();
  aorrt.link_and_setup_query(&aorrt_query);

  prx::condition_check_t checker(params["checker_type"].as<>(), params["checker_value"].as<int>());

  PRX_DBG_VARS(aorrt_query.start_state, aorrt_query.goal_state);
  aorrt.resolve_query(&checker);
  aorrt.fulfill_query();

  prx::three_js_group_t* vis_group = new prx::three_js_group_t({ plant }, { obstacle_list });

  std::shared_ptr<prx::tree_t> slns_tree{ aorrt.tree_of_solutions() };
  std::vector<prx::trajectory_t> slns_trajs{ prx::rrt_t::tree_to_trajectories<prx::aorrt_edge_t>(*slns_tree) };

  const std::string body_name{ plant_name + "/" + "body" };

  vis_group->set_floor_plane(Eigen::Vector3d(0, 0, -0.1), Eigen::Vector4d(0, 0, 0, 1), Eigen::Vector2d(0, 1),
                             "0x000000");
  vis_group->add_vis_infos(prx::info_geometry_t::LINE, slns_trajs, body_name, ss);
  vis_group->add_detailed_vis_infos(prx::info_geometry_t::FULL_LINE, aorrt_query.solution_traj, body_name, ss);
  vis_group->add_animation(aorrt_query.solution_traj, ss, aorrt_query.start_state);

  gtsam::Values values;
  gtsam::NonlinearFactorGraph graph;

  auto edges_iters = slns_tree->edges();

  gtsam::noiseModel::Base::shared_ptr noise_integrator_model{ gtsam::noiseModel::Isotropic::Sigma(3, 1) };
  gtsam::noiseModel::Base::shared_ptr Q_noise_model{ gtsam::noiseModel::Isotropic::Sigma(3, 1) };
  gtsam::noiseModel::Base::shared_ptr R_noise_model{ gtsam::noiseModel::Isotropic::Sigma(3, 1) };

  std::vector<gtsam::Key> Xkeys, Ukeys;
  for (auto iter = edges_iters.first; iter != edges_iters.second; iter++)
  {
    const std::shared_ptr<prx::aorrt_edge_t> edge{ std::dynamic_pointer_cast<prx::aorrt_edge_t>(*iter) };
    const std::size_t parent_id{ edge->get_source() };
    const std::size_t target_id{ edge->get_target() };

    const gtsam::Key key_xt0{ SF::create_hashed_symbol("X^{", parent_id, "}") };
    const gtsam::Key key_xt1{ SF::create_hashed_symbol("X^{", target_id, "}") };
    const gtsam::Key key_xdot{ SF::create_hashed_symbol("U^{", parent_id, "}_{", target_id, "}") };

    Xkeys.push_back(key_xt0);
    Ukeys.push_back(key_xdot);

    const double duration{ edge->plan->duration() };
    const prx::fg::SE2_t x0{ Vec(edge->traj->front()) };
    const prx::fg::SE2_t x1{ Vec(edge->traj->back()) };

    const Eigen::Vector3d xdot{ prx::fg::SE2_t::Logmap(x0.between(x1)) / duration };
    // PRX_DBG_VARS(xdot.transpose(), edge->plan);

    values.insert_or_assign(key_xt0, x0);
    values.insert_or_assign(key_xt1, x1);
    values.insert_or_assign(key_xdot, xdot);

    graph.emplace_shared<Integrator>(key_xt1, key_xt0, key_xdot, noise_integrator_model, duration);

    graph.addPrior(key_xt0, x0, Q_noise_model);
    graph.addPrior(key_xdot, xdot, R_noise_model);

    if (slns_tree->is_leaf(target_id))
    {
      Xkeys.push_back(key_xt1);
      graph.addPrior(key_xt1, x1, Q_noise_model);
    }
  }

  gtsam::LevenbergMarquardtParams lm_params{ prx::fg::levenberg_marquardt_parameters(params["FG/LM"]) };

  gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
  gtsam::Values result{ optimizer.optimize() };

  std::vector<Eigen::Vector3d> fg_trajs;
  for (auto iter = edges_iters.first; iter != edges_iters.second; iter++)
  {
    const std::shared_ptr<prx::tree_edge_t> edge{ *iter };
    const std::size_t idx{ edge->get_index() };

    const gtsam::Key key_xt{ SF::create_hashed_symbol("X^{", idx, "}") };

    const prx::fg::SE2_t xt{ result.at<prx::fg::SE2_t>(key_xt) };
    fg_trajs.emplace_back(xt[0], xt[1], xt[2]);
  }

  vis_group->add_vis_infos(prx::info_geometry_t::LINE, fg_trajs, "0x00ff00");
  vis_group->output_html("lqr_tree.html");

  std::map<gtsam::Key, Eigen::Matrix<double, 3, 3>> Ks;  // u = -k X ==>  (3x1)= (3x3) (3x1)
  std::map<gtsam::Key, Eigen::Matrix<double, 3, 3>> Ss;

  auto linearized_graph = graph.linearize(result);
  compute_K_S(linearized_graph, Xkeys, Ukeys, Ss, Ks);

  std::ofstream ofs_Kfile(params["out/Kfile"].as<>().c_str());
  std::ofstream ofs_Sfile(params["out/Sfile"].as<>().c_str());

  tree_to_file(params["out/tree"].as<>(), slns_tree);

  // slns_tree->to_file(params["out/tree"].as<>());

  for (auto Kks : Ks)
  {
    const std::string strKey{ SF::formatter(Kks.first) };

    Eigen::MatrixXd mat{ Kks.second };
    const prx::fg::SE2_t xi{ result.at<prx::fg::SE2_t>(Kks.first) };
    // PRX_DBG_VARS(strKey, mat);

    mat.resize(1, mat.rows() * mat.cols());

    ofs_Kfile << strKey << " " << xi << " " << mat << "\n";
  }
  ofs_Kfile.close();

  for (auto Si : Ss)
  {
    const std::string strKey{ SF::formatter(Si.first) };

    Eigen::MatrixXd mat{ Si.second };
    const prx::fg::SE2_t xi{ result.at<prx::fg::SE2_t>(Si.first) };
    // PRX_DBG_VARS(strKey, mat);

    mat.resize(1, mat.rows() * mat.cols());

    ofs_Sfile << strKey << " " << xi << " " << mat << "\n";
  }
  ofs_Sfile.close();

  delete vis_group;
}
