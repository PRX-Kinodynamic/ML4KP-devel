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
#include "prx/utilities/geometry/regular_grid.hpp"
#include "prx/utilities/general/range.hpp"

#include "prx/visualization/three_js_group.hpp"

#include "prx/factor_graphs/defs.hpp"
#include "prx/factor_graphs/utilities/fg_logger.hpp"
#include "prx/factor_graphs/planning/initialization_trajs_fg.hpp"
// #include "prx/factor_graphs/planning/trajectory_fg.hpp"
// #include "prx/factor_graphs/planning/trajectory_optimizer.hpp"

#include "prx/factor_graphs/utilities/utilities_functions.hpp"
#include "prx/factor_graphs/utilities/formatter.hpp"

#include <gtsam/nonlinear/Marginals.h>

namespace fs = std::filesystem;
using namespace prx;
using friction_vector_t = Eigen::Vector<double, 1>;
// using friction_vector_t = Eigen::Vector4d;

gtsam::LevenbergMarquardtParams fg_params()
{
  gtsam::LevenbergMarquardtParams lm_params;
  lm_params.setVerbosityLM("SUMMARY");
  lm_params.setlambdaUpperBound(1e32);
  lm_params.setUseFixedLambdaFactor(false);
  lm_params.setDiagonalDamping(true);
  lm_params.setlambdaFactor(2);
  lm_params.setlambdaInitial(1e-7);
  lm_params.setMaxIterations(10);
  lm_params.setRelativeErrorTol(1e-6);
  lm_params.setAbsoluteErrorTol(1e-6);

  // const std::string log_file{ out_path + "friction_maps/fg_concurrent_log.txt" };
  // std::remove(log_file.c_str());
  // lm_params.setLogFile(log_file);

  return lm_params;
}

void init_ground_truth_grid(prx::regular_grid_t<double, 2>& gt_grid, const int x_max, const int y_max, const int div)
{
  const double min_val{ 0.5 };
  const double max_val{ 2.0 };
  const double x_step{ x_max / static_cast<double>(div) };
  const double y_step{ y_max / static_cast<double>(div) };
  std::vector<double> vals{ linspace(min_val, max_val, 1 + div / 2) };
  vals.insert(vals.end(), vals.rbegin() + 1, vals.rend());
  for (double x = 0; x < x_max; x += x_step)
  {
    int i = 0;
    for (double y = 0; y < y_max; y += y_step)
    {
      gt_grid(x, y) = vals[i];
      i++;
    }
  }
  // gt_grid.set_to(2.0);
}

template <typename grid1_t, typename grid2_t, typename log_t, typename Priors, typename Ids>
void compute_grid_error(const grid1_t& frictions_grid, const grid2_t& gt_grid, log_t& log, const int iter,
                        const Priors& priors, Ids& ids)
{
  grid1_t error_grid{ frictions_grid };
  // std::cout << error_grid << std::endl;
  for (auto cell : gt_grid)
  {
    auto unmap_vals = gt_grid.template unmap_key<Eigen::Vector2d>(cell.first);
    const prx_symbol_t theta_simbol{ symbol_factory_t::create_symbol("param_symbol",
                                                                     ids(unmap_vals[0], unmap_vals[1])) };
    if (priors.find(theta_simbol) == priors.end())
    {
      error_grid[cell.first].array() *= 0;
    }
    else
    {
      error_grid[cell.first].array() -= cell.second;
    }
  }
  // std::cout << error_grid << std::endl;
  using error_grid_t = decltype(*(error_grid.begin()));
  friction_vector_t grid_error{ friction_vector_t::Zero() };
  grid_error = std::accumulate(error_grid.begin(), error_grid.end(), grid_error,
                               [](auto a, error_grid_t& b) { return a + b.second.cwiseAbs(); });
  // double sqr_error = std::accumulate(error_grid.begin(), error_grid.end(), 0.0,
  if (grid_error.sum() > 1e100)
  {
    std::cout << "error_grid: " << error_grid << std::endl;
    std::cout << "error: " << grid_error.transpose() << std::endl;
  }
  log.log(iter, grid_error.sum());
}

template <typename Ids, typename Frictions, typename Priors>
gtsam::NonlinearFactorGraph compute_thetas_graph(const double x_max, const double y_max, const double grid_stepping,
                                                 Ids& grid, Frictions& frictions, const space_t* ps,
                                                 const gtsam::Values& results, gtsam::Values& vals, fg_logger_t& logger,
                                                 const int iter, const Priors& priors)
{
  gtsam::NonlinearFactorGraph theta_graph;
  std::unordered_map<int, bool> added;
  for (double x = 0; x < x_max; x += grid_stepping)
  {
    for (double y = 0; y < y_max; y += grid_stepping)
    {
      if (added.find(grid(x, y)) == added.end())
      {
        const prx_symbol_t theta_simbol{ symbol_factory_t::create_symbol("param_symbol", grid(x, y)) };
        theta_graph.add(
            space_limit_factor_t<1>(theta_simbol, gtsam::noiseModel::Isotropic::Sigma(ps->get_dimension(), 1e0), ps));
        Eigen::VectorXd theta{ Eigen::VectorXd::Ones(ps->get_dimension()) };
        if (results.exists(theta_simbol))
        {
          // std::cout << prx::key_formatter(theta_simbol) << std::endl;
          theta = results.at<Eigen::VectorXd>(theta_simbol);
          // theta_graph.addPrior(theta_simbol, theta, marginals.marginalCovariance(theta_simbol));
        }
        // Eigen::VectorXd param_guess{ frictions_grid(x, y) };
        // init_vals.insert(symbol, param_guess);

        vals.insert_or_assign(theta_simbol, theta);
        frictions(x, y) = theta;

        logger.log(iter, x, y, theta.transpose(), (priors.find(theta_simbol) != priors.end() ? 1 : 0));
        added[grid(x, y)] = true;
      }
    }
  }
  for (auto pair : priors)
  {
    const prx_symbol_t theta_simbol{ pair.first };
    // std::cout << prx::key_formatter(theta_simbol) << std::endl;

    Eigen::VectorXd theta = results.at<Eigen::VectorXd>(theta_simbol);
    theta_graph.addPrior(theta_simbol, theta, pair.second);
  }
  return theta_graph;
}

int main(int argc, char* argv[])
{
  auto params = param_loader("executables/factor_graphs/friction_map_concurrent.yaml", argc, argv);

  simulation_step = params["simulation_step"].as<double>();
  init_random(params["random_seed"].as<int>());

  std::string plant_name = params["/plant/name"].as<>();
  std::string plant_path = params["/plant/path"].as<>();

  auto obstacles = load_obstacles(params["environment"].as<>());

  auto obstacle_list = obstacles.second;
  auto obstacle_names = obstacles.first;

  auto system = system_factory_t::create_system(plant_name, plant_path);
  auto plant = std::dynamic_pointer_cast<plant_t>(system);
  world_model_t world_model({ plant }, {});
  world_model.create_context("context", { plant_name }, {});

  auto context = world_model.get_context("context");
  auto sg = context.first;
  const auto ss = sg->get_state_space();
  const auto cs = sg->get_control_space();
  const auto ps = sg->get_parameter_space();
  const auto ss_dim = ss->get_dimension();
  const auto cs_dim = cs->get_dimension();
  const auto ps_dim = ps->get_dimension();
  prx_assert(ps != nullptr, "Parameter space is null!!!");

  const double x_max{ 3.0 };
  const double y_max{ 3.0 };
  ss->set_bounds({ 0.0, 0.0, -M_PI }, { x_max, y_max, M_PI });

  auto cs_lb = params["/plant/control_space_lower_bound"].as<std::vector<double>>();
  auto cs_up = params["/plant/control_space_upper_bound"].as<std::vector<double>>();
  cs->set_bounds(cs_lb, cs_up);

  const std::string fm_out_dir{ out_path + "friction_maps/" };
  const std::string traj_real_file{ fm_out_dir + "concurrent_analytical_fixed_goals_trajs_real.txt" };
  const std::string plan_real_file{ fm_out_dir + "concurrent_analytical_fixed_goals_plans_real.txt" };
  const std::string traj_fg_file{ fm_out_dir + "concurrent_analytical_fixed_goals_trajs_fg.txt" };
  const std::string fg_graph_file{ fm_out_dir + "concurrent_factor_graph.dot" };

  std::remove(traj_real_file.c_str());
  std::remove(plan_real_file.c_str());
  std::remove(traj_fg_file.c_str());
  std::remove(fg_graph_file.c_str());

  std::ofstream ofs_gt_frmap, ofs_plans, ofs_goals, ofs_traj_real;
  ofs_goals.open(fm_out_dir + "concurrent_analytical_fixed_goals.txt", std::ofstream::trunc);
  ofs_gt_frmap.open(fm_out_dir + "concurrent_friction_map_fixed_goals.txt", std::ofstream::trunc);
  // friction_map_file.open(fm_out_dir + "concurrent_fg_friction_map.txt");
  fg_logger_t friction_map_logger(fm_out_dir + "concurrent_fg_friction_map.txt", ' ', "-");

  // const std::vector<std::pair<double, double>> env_bounds{ std::make_pair(0.0, 3.0), std::make_pair(0.0, 3.0) };
  const int divisions{ 10 };
  const std::vector<std::pair<double, double>> env_bounds{ std::make_pair(0.0, x_max), std::make_pair(0.0, y_max) };
  prx::regular_grid_t<double, 2> gt_grid{ env_bounds, divisions };
  init_ground_truth_grid(gt_grid, x_max, y_max, divisions);

  prx::regular_grid_t<friction_vector_t, 2> frictions_grid{ env_bounds, divisions };

  bool write_to_file = false;
  // bool real_friction = true;
  Eigen::Vector4d fg_friction_maps;
  // world_model.world_change_function = [&]() {
  friction_vector_t friction_params{};
  std::function<void()> ground_truth_world = [&]() {
    const double x{ ss->at(0) };
    const double y{ ss->at(1) };
    const double th{ ss->at(2) };
    double x1, x2, x3, x4;
    double y1, y2, y3, y4;
    x1 = x2 = x3 = x4 = x;
    y1 = y2 = y3 = y4 = y;
    // const double l_a{ 0.11 };
    // const double l_b{ 0.10 };
    // const double x1{ x + l_a * std::cos(th) - l_b * std::sin(th) };
    // const double y1{ y + l_a * std::sin(th) + l_b * std::cos(th) };
    // const double x2{ x + (-l_a) * std::cos(th) - l_b * std::sin(th) };
    // const double y2{ y + (-l_a) * std::sin(th) + l_b * std::cos(th) };
    // const double x3{ x + l_a * std::cos(th) - (-l_b) * std::sin(th) };
    // const double y3{ y + l_a * std::sin(th) + (-l_b) * std::cos(th) };
    // const double x4{ x + (-l_a) * std::cos(th) - (-l_b) * std::sin(th) };
    // const double y4{ y + (-l_a) * std::sin(th) + (-l_b) * std::cos(th) };
    friction_params = friction_vector_t{ gt_grid(x1, y1) };
    // friction_params[1] = gt_grid(x2, y2);
    // friction_params[2] = gt_grid(x3, y3);
    // friction_params[3] = gt_grid(x4, y4);
    if (write_to_file)
    {
      // std::cout << friction_params.transpose() << std::endl;
      ofs_gt_frmap << x1 << " " << y1 << " " << friction_params[0] << "\n";
      // ofs_gt_frmap << x2 << " " << y2 << " " << friction_params[1] << "\n";
      // ofs_gt_frmap << x3 << " " << y3 << " " << friction_params[2] << "\n";
      // ofs_gt_frmap << x4 << " " << y4 << " " << friction_params[3] << "\n";
    }
    if (x < 0.0 || x_max < x || y < 0.0 || y_max < y)
    {
      friction_params = friction_vector_t::Ones();
    }
    ps->copy_from(friction_params);
  };
  std::function<void()> fg_world = [&]() {};
  std::function<void()> testing_world = [&]() {
    const double x{ ss->at(0) };
    const double y{ ss->at(1) };
    if (x < 0.0 || x_max < x || y < 0.0 || y_max < y)
    {
      friction_params = friction_vector_t::Ones();
    }
    else
    {
      friction_params = friction_vector_t{ gt_grid(x, y) };
      // friction_params[1] = gt_grid(x, y);
      // friction_params[2] = gt_grid(x, y);
      // friction_params[3] = gt_grid(x, y);
    }
    // std::cout << x << ", " << y << "\tfriction_params: " << friction_params.transpose() << std::endl;
    ps->copy_from(friction_params);
    ps->enforce_bounds();
  };

  write_to_file = true;
  world_model.world_change_function = ground_truth_world;
  const double grid_stepping{ 1.0 / static_cast<double>(divisions) };
  for (double i = 0; i < x_max; i += grid_stepping)
  {
    for (double j = 0; j < y_max; j += grid_stepping)
    {
      ss->copy_from(std::vector{ i, j, 0.0 });
      world_model.world_change_function();
    }
  }
  write_to_file = false;
  // return 0;

  std::vector<std::vector<double>> goals = { { 0.1 * x_max, 0.1 * y_max, 0 } };
  // std::vector<std::vector<double>> goals;  // = { { 0.1 * x_max, 0.1 * y_max, 0 }, { 0.9 * x_max, 0.9 * y_max, 0 },
  //    { 0.9 * x_max, 0.1 * y_max, 0 }, { 0.1 * x_max, 0.9 * y_max, 0 },
  //    { 0.5 * x_max, 0.1 * y_max, 0 }, { 0.5 * x_max, 0.9 * y_max, 0 } };

  for (int i = 0; i < 150; ++i)
  {
    double x_new{ x_max * uniform_random() };
    double y_new{ y_max * uniform_random() };
    double dist = std::sqrt(std::pow(goals.back()[0] - x_new, 2) + std::pow(goals.back()[1] - y_new, 2));
    if (dist < 1)
    {
      i--;
    }
    else
    {
      goals.push_back({ x_new, y_new, 0 });
    }
  }

  std::shared_ptr<custom_controller_t> omnibot_controller =
      std::make_shared<custom_controller_t>(plant, "omnibot_controller");

  const double l_a = .11;
  const double l_b = .10;
  const double l_ab = l_a + l_b;

  Eigen::MatrixXd inverse(4, 3);
  inverse << -1.0 / (4.0 * l_ab), 1.0 / (4.0 * l_ab), 1.0 / 4.0,  // no-lint
      1.0 / (4.0 * l_ab), 1.0 / (4.0 * l_ab), -1.0 / 4.0,         // no-lint
      -1.0 / (4.0 * l_ab), 1.0 / (4.0 * l_ab), -1.0 / 4.0,        // no-lint
      1.0 / (4.0 * l_ab), 1.0 / (4.0 * l_ab), 1.0 / 4.0;          // no-lint

  Eigen::Vector3d current_state_vec;
  const double frequency{ params["frequency"].as<double>() };
  double curr_freq{ frequency };

  auto n_plant = new prx::noisy_plant_t<prx::gaussian_noise_t>(plant, 0, params["noise_var"].as<double>());
  auto n_ss = n_plant->get_state_space();

  Eigen::Vector4d U{ Eigen::Vector4d::Zero() };
  const double goal_region_radius{ params["goal_region_radius"].as<double>() };
  // clang-format off
  omnibot_controller -> custom_control_function = [&](const space_point_t& goal, const space_point_t& control) 
  {
    curr_freq += simulation_step;
    n_ss -> copy_to(current_state_vec);
    const Eigen::Vector3d xd{ goal->vector() - current_state_vec};
    if (xd.norm() < goal_region_radius)
    {
      U =  Eigen::Vector4d::Zero();
    }
    else if (curr_freq >= frequency)
    {
      U = (inverse * xd).normalized() * 128;
      curr_freq = 0;
    }
    cs -> copy(control, U);
  };
  auto compute_fg = [&](const trajectory_t& traj, const plan_t& plan)
  {
  };
  // clang-format on

  trajectory_t traj_real(ss);
  trajectory_t traj_fg(ss);
  trajectory_t accum_traj(ss);
  traj_real.clear();
  traj_fg.clear();
  accum_traj.clear();
  plan_t accum_plan(cs);

  const int num_trajs{ params["num_trajs"].as<int>() };

  space_point_t goal = ss->make_point();
  condition_check_t checker("sim_time", 1);
  auto cc = create_default_goal_check(ss, goal, goal_region_radius);
  // condition_check_t check_goal_reached(cc);
  // checker.add_condition(&check_goal_reached);

  // Factor graphs
  // gtsam::NonlinearFactorGraph graph;
  // gtsam::NonlinearFactorGraph graph_theta;
  gtsam::LevenbergMarquardtParams lm_params{ fg_params() };
  gtsam::Values init_vals;
  gtsam::Values results;

  prx::regular_grid_t<int, 2> fg_grid{ env_bounds, divisions };
  int cell_number{ 0 };
  for (double x = 0; x < x_max; x += grid_stepping)
  {
    for (double y = 0; y < y_max; y += grid_stepping)
    {
      fg_grid(x, y) = cell_number;
      frictions_grid(x, y) = friction_vector_t::Ones() * params["initial_friction"].as<double>();
      //: Eigen::Vector4d::Random() * 2.0;
      // const prx_symbol_t symbol{ symbol_factory_t::create_symbol("param_symbol", cell_number) };
      // graph_theta.add(space_limit_factor_t(symbol, gtsam::noiseModel::Isotropic::Sigma(ps_dim, 1e0), ps));

      // Eigen::VectorXd param_guess{ frictions_grid(x, y) };
      // init_vals.insert(symbol, param_guess);

      cell_number++;
    }
  }

  std::unordered_map<prx_symbol_t, Eigen::MatrixXd> theta_priors;

  gtsam::NonlinearFactorGraph graph_theta{ compute_thetas_graph(x_max, y_max, grid_stepping, fg_grid, frictions_grid,
                                                                ps, results, init_vals, friction_map_logger, -1,
                                                                theta_priors) };

  const int increment = static_cast<int>(frequency / simulation_step);
  std::cout << "increment: " << increment << std::endl;
  auto x_sigma = gtsam::noiseModel::Isotropic::Sigma(ss_dim, 1e-3);
  auto dm = gtsam::noiseModel::Isotropic::Sigma(ss_dim, 1e-0);
  auto cs_dm = gtsam::noiseModel::Isotropic::Sigma(cs_dim, 1e-3);
  auto t_dm = gtsam::noiseModel::Isotropic::Sigma(1, 1e-3);

  fg::formatter_t graph_formatter;

  fg_logger_t lg(out_path + "friction_maps/fg_concurrent_log.txt", ' ', "-");
  logger_t grid_error_lg(out_path + "friction_maps/fg_concurrent_error_grid_log.txt", ' ');
  int fg_iters{ 0 };
  space_point_t start_state = ss->make_point();
  const int initial_goal{ params["initial_goal"].as<int>() };
  ss->copy(start_state, goals[initial_goal % goals.size()]);
  for (int i = initial_goal; i < num_trajs; ++i)
  {
    std::cout << "Going into trajectory: " << i << "..." << std::endl;

    compute_grid_error(frictions_grid, gt_grid, grid_error_lg, fg_iters, theta_priors, fg_grid);
    curr_freq = frequency;

    traj_real.clear();
    traj_fg.clear();
    checker.reset();
    omnibot_controller->get_plan()->clear();

    ss->copy_from(start_state);
    if (cc())
    {
      // ss->copy(start_state, goals[i % goals.size()]);
      // n_ss->add_noise(start_state);
      ss->copy(goal, goals[(i + 1) % goals.size()]);
      omnibot_controller->set_goal(goal);
    }

    std::cout << "start_state: " << start_state << std::endl;

    PRX_DEBUG_PRINT
    // real_friction = true;
    world_model.world_change_function = ground_truth_world;
    sg->propagate(start_state, omnibot_controller, checker, traj_real);

    std::cout << "traj_real: " << traj_real.size() << std::endl;
    const plan_t plan{ *(omnibot_controller->get_plan()) };
    std::cout << "plan size: " << plan.size() << std::endl;
    gtsam::NonlinearFactorGraph graph_trajs;
    std::set<prx_symbol_t> thetas_used;
    for (unsigned xi = 0; xi < traj_real.size(); xi += increment)
    {
      if (xi < traj_real.size() - increment - 1)
      {
        const double x{ traj_real[xi]->vector()[0] };
        const double y{ traj_real[xi]->vector()[1] };
        const double x1{ traj_real[xi + increment]->vector()[0] };
        const double y1{ traj_real[xi + increment]->vector()[1] };

        auto state_symbol = symbol_factory_t::create_symbol("state_symbol", i, xi);
        auto next_state_symbol = symbol_factory_t::create_symbol("state_symbol", i, xi + increment);
        auto control_symbol = symbol_factory_t::create_symbol("control_symbol", i, xi);
        auto time_symbol = symbol_factory_t::create_symbol("time_symbol", i, xi);
        auto param_symbol = symbol_factory_t::create_symbol("param_symbol", fg_grid(x, y));
        if (fg_grid(x, y) != fg_grid(x1, y1))
        {
          // graph_trajs.addPrior(next_state_symbol, traj_real[xi + increment]->vector<>(), x_sigma);
          // init_vals.insert(next_state_symbol, traj_real[xi + increment]->vector<>());
          graph_trajs.addPrior(state_symbol, traj_real[xi]->vector<>(), x_sigma);
          init_vals.insert(state_symbol, traj_real[xi]->vector<>());
          continue;
        }

        thetas_used.insert(param_symbol);
        Eigen::VectorXd t_vec{ (Eigen::VectorXd(1) << plan[xi].duration * increment).finished() };
        graph_trajs.addPrior(state_symbol, traj_real[xi]->vector<>(), x_sigma);
        graph_trajs.addPrior(control_symbol, plan[xi].control->vector<>(), cs_dm);
        graph_trajs.addPrior(time_symbol, t_vec, t_dm);

        init_vals.insert(state_symbol, traj_real[xi]->vector<>());
        init_vals.insert(control_symbol, plan[xi].control->vector<>());
        init_vals.insert(time_symbol, t_vec);
        if (xi + increment >= traj_real.size() - increment - 1)
        {
          graph_trajs.addPrior(next_state_symbol, traj_real[xi + increment]->vector<>(), x_sigma);
          init_vals.insert(next_state_symbol, traj_real[xi + increment]->vector<>());
        }
        graph_trajs.add(propagation_factor_5_t<3, 4, 1>(state_symbol, next_state_symbol, control_symbol, time_symbol,
                                                        param_symbol, dm, sg));

        // graph_trajs.add(propagation_factor_1_t(dm, traj_real[xi]->vector<>(), traj_real[xi +
        // increment]->vector<>(),
        //                                  plan.at(xi).control->vector<>(),
        //                                  Eigen::Vector<double, 1>{ plan.at(xi).duration * increment },
        //                                  symbol_factory_t::create_symbol("param_symbol", fg_grid(x, y)), sg));

        // }
      }
    }
    gtsam::NonlinearFactorGraph graph;
    graph.add(graph_theta);
    graph.add(graph_trajs);
    world_model.world_change_function = fg_world;
    std::cout << "Graph: " << graph.size() << std::endl;
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_vals, lm_params);
    auto results = fg::utilities::optimize_and_log(optimizer, lm_params, lg, fg_iters);

    graph.saveGraph(fg_graph_file, results, prx::key_formatter, graph_formatter);
    gtsam::Marginals marginals{ graph_trajs, results };
    for (auto theta_u : thetas_used)
    {
      // std::cout << prx::key_formatter(theta_u) << " Inf: " << marginals.marginalInformation(theta_u)
      //           << " Cov: " << marginals.marginalCovariance(theta_u) << std::endl;
      theta_priors[theta_u] = marginals.marginalCovariance(theta_u);
    }
    fg_iters += optimizer.iterations();

    graph_theta = compute_thetas_graph(x_max, y_max, grid_stepping, fg_grid, frictions_grid, ps, results, init_vals,
                                       friction_map_logger, fg_iters, theta_priors);

    std::cout << "init_vals size: " << init_vals.size() << std::endl;
    world_model.world_change_function = ground_truth_world;
    sg->propagate(start_state, plan, traj_fg);
    std::cout << "traj_fg: " << traj_fg.size() << std::endl;
    traj_fg.to_file(traj_fg_file, std::ofstream::app);
    ss->copy(start_state, traj_real.back());

    accum_traj += traj_real;
    accum_plan += plan;
  }
  for (auto pair : theta_priors)
  {
    std::cout << prx::key_formatter(pair.first) << ": " << pair.second << std::endl;
  }
  std::cout << "Finishing..." << std::endl;
  compute_grid_error(frictions_grid, gt_grid, grid_error_lg, fg_iters, theta_priors, fg_grid);

  // std::cout << "grid_error: " << sqr_error << std::endl;
  // std::cout << "grid_error: " << grid_error.transpose() << std::endl;
  // This are not "Real" trajectories/plan, is all appended into one and might
  // have discontinuities. Only used for dumping into a file.
  accum_traj.to_file(traj_real_file, std::ofstream::app);
  accum_plan.to_file(plan_real_file, std::ofstream::app);
}