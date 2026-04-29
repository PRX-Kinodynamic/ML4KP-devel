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
#include "prx/utilities/general/csv_reader.hpp"

using csv_reader_t = prx::utilities::csv_reader_t;
using prx::utilities::convert_to;

template <typename MatOut, typename InVec>
MatOut mat_from_vec(const InVec& v, const std::size_t& first, const std::size_t& last = -1)
{
  MatOut mat{ MatOut::Zero() };
  std::size_t i{ 0 };
  for (double& e : mat.reshaped())
  {
    e = convert_to<double>(v[first + i]);
    if (i == last)
      break;
    i++;
  }
  return mat;
}

template <typename KeyX, typename KeyMat>
void read_lqr_file(const std::string file, KeyX& keysX, KeyMat& keysMat)
{
  prx::fg::SE2_t xi;

  csv_reader_t reader(file, ' ');
  while (reader.has_next_line())
  {
    auto line = reader.next_line();

    if (line.size() == 0)
      continue;

    const std::string key{ line[0] };
    const prx::fg::SE2_t xi{ mat_from_vec<Eigen::Vector3d>(line, 1, 3) };
    const Eigen::Matrix3d Mi{ mat_from_vec<Eigen::Matrix3d>(line, 4) };

    // PRX_DBG_VARS(key, xi);
    // PRX_DBG_VARS(key, Mi);
    keysX[key] = xi;
    keysMat[key] = Mi;
  }
}

template <typename KeyX, typename KeyS>
std::string find_closest(const prx::fg::SE2_t& xi, KeyX& Xs, KeyS& Ss, const double epsilon = 1)
{
  using PairKeyCost = std::pair<std::string, double>;
  auto compF = [](const PairKeyCost& p1, const PairKeyCost& p2) { return p1.second > p2.second; };

  std::priority_queue<PairKeyCost, std::vector<PairKeyCost>, decltype(compF)> pq_keys{ compF };

  for (auto pair_key_state : Xs)
  {
    const std::string& key{ pair_key_state.first };
    const Eigen::Matrix3d Si{ Ss[key] };
    const prx::fg::SE2_t Xl{ Xs[key] };
    const double cost{ prx::fg::lie_operators::cost_to_go(xi, Si, Xl) };
    pq_keys.push(std::make_pair(key, cost));
  }

  if (pq_keys.top().second < 1)
  {
    pq_keys.pop();
  }
  return pq_keys.top().first;
}

template <typename EdgeMap>
void read_tree(const std::string tree_filename, EdgeMap& edges)
{
  csv_reader_t reader(tree_filename, ' ');

  while (reader.has_next_line())
  {
    auto line = reader.next_line();
    if (line.size() == 0)
      continue;
    const std::string key_source{ line[0] };
    const std::string key_target{ line[1] };

    edges[key_source].push_back(key_target);
  }
}

template <typename KeyX, typename KeyS, typename KeysKs, typename Branch>
prx::fg::SE2_t fwd_prop(prx::fg::SE2_t xi, std::shared_ptr<prx::system_group_t> sg, prx::condition_check_t& checker,
                        KeyX& Xs, KeyS& Ss, KeysKs& Ks, Branch& branch, double epsilon)
{
  // prx::fg::SE2_t xi{ Vec(pt) };
  std::string key{ find_closest(xi, Xs, Ss) };

  checker.reset();
  Eigen::Vector3d ui{ Eigen::Vector3d::Zero() };

  prx::fg::SE2_t xgoal{ Xs[key] };

  // PRX_DBG_VARS(key, xgoal);
  prx::space_t* ss{ sg->get_state_space() };
  prx::space_t* cs{ sg->get_control_space() };
  do
  {
    ss->copy_to(xi);
    Eigen::Vector3d tau{ prx::fg::lie_operators::right_minus(xgoal, xi) };
    if (tau.norm() < epsilon)
    {
      key = branch[key];
      xgoal = Xs[key];
      tau = prx::fg::lie_operators::right_minus(xgoal, xi);
      // PRX_DBG_VARS(key, xgoal);
    }

    ui = Ks[key] * tau;
    // PRX_DBG_VARS(xi);
    // PRX_DBG_VARS(tau.transpose(), ui.transpose());
    cs->copy_from(ui);

    sg->propagate_once();
    // result.copy_onto_back(state_space);
  } while (!checker.check());

  prx::fg::SE2_t result;
  sg->get_state_space()->copy_to(result);

  return result;
}

template <typename Edges, typename Goal, typename Branch>
void find_branch(Edges edges, Goal goal, Branch branch)
{
  branch.clear();
  // For now the tree is a single branch, so just adapt it:
  for (auto pair : edges)
  {
    branch[pair.first] = pair.second.front();
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
  params["state_space/dim"].set(3);
  params["Kfile"].set(prx::out_path + "/se2_kfile.txt");
  params["Sfile"].set(prx::out_path + "/se2_sfile.txt");
  params["symbol"].set("X^{0}");
  params["TreeFile"].set(prx::out_path + "/lqr_tree.txt");

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

  prx::space_point_t pt{ ss->make_point(params["query/start_state"]) };

  std::map<std::string, prx::fg::SE2_t> Xs;
  std::map<std::string, Eigen::Matrix<double, 3, 3>> Ks;
  std::map<std::string, Eigen::Matrix<double, 3, 3>> Ss;
  std::map<std::string, std::vector<std::string>> edges;
  std::map<std::string, std::string> branch;

  read_lqr_file(params["Kfile"].as<>(), Xs, Ks);
  read_lqr_file(params["Sfile"].as<>(), Xs, Ss);
  read_tree(params["TreeFile"].as<>(), edges);

  const std::string symbol{ params["symbol"].as<>() };
  Eigen::Matrix3d Si{ Ss[symbol] };

  prx::fg::SE2_t xi{ 0, 0, 0 };
  prx::fg::SE2_t xg{ Xs[symbol] };

  std::ofstream ofs_gt(prx::out_path + "/gt.txt");

  for (auto pair_key_state : Xs)
  {
    const std::string key{ pair_key_state.first };
    ofs_gt << key << " " << Xs[key] << "\n";
  }
  ofs_gt.close();

  std::ofstream ofs_file(prx::out_path + "/cost_map.txt");

  find_branch(edges, xg, branch);

  const double epsilon{ 0.1 };
  prx::fg::SE2_t result;
  prx::condition_check_t checker("sim_time", 1);

  result = fwd_prop(xi, sg, checker, Xs, Ss, Ks, branch, epsilon);

  PRX_DBG_VARS(result);
  for (double x = -5; x < 25; x += 0.1)
  {
    xi[0] = x;
    for (double y = -5; y < 25; y += 0.1)
    {
      xi[1] = y;
      // find_closest
      const std::string key{ find_closest(xi, Xs, Ss) };
      const prx::fg::SE2_t Xclose{ Xs[key] };
      const double cost{ prx::fg::lie_operators::cost_to_go(xi, Si, xg) };

      result = fwd_prop(xi, sg, checker, Xs, Ss, Ks, branch, epsilon);

      ofs_file << xi << " ";
      ofs_file << cost << " ";
      ofs_file << Xclose << " ";
      ofs_file << result << " ";
      ofs_file << "\n";
    }
  }

  // Eigen::Vector2d ui;
  // ss->copy_from(pt);
  // int i = 0;
  // do
  // {
  //   const prx::fg::SE2_t xi{ Vec(pt) };
  //   const std::string key{ find_closest(xi, Xs, Ss) };
  //   ui = -Ks[key] * right_minus(xi, );

  //       propagate_once(nullptr);
  //   result.copy_onto_back(state_space);
  // } while (!cond_check.check());

  ofs_file.close();
}
