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
#include "prx/utilities/math/first_order_derivative.hpp"

#include "prx/visualization/three_js_group.hpp"

#include "prx/factor_graphs/defs.hpp"
#include "prx/factor_graphs/utilities/fg_logger.hpp"
// #include "prx/factor_graphs/planning/initialization_trajs_fg.hpp"
// #include "prx/factor_graphs/planning/trajectory_fg.hpp"
// #include "prx/factor_graphs/planning/trajectory_optimizer.hpp"
#include "prx/factor_graphs/factors/parameter_fusion_factor.hpp"

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

template <class thetas_grid_t>
gtsam::NonlinearFactorGraph find_closest_thetas(const std::size_t& t, const prx_symbol_t& xt_symbol,
                                                const prx_symbol_t& param_t, const Eigen::Vector3d& x_in,
                                                const thetas_grid_t& thetas_grid, gtsam::Values& values)
{
  std::vector<Eigen::Vector2d> xi;
  xi.emplace_back(x_in[0], x_in[1]);
  xi.emplace_back(x_in[0] + thetas_grid.get_cell_length(0), x_in[1]);
  xi.emplace_back(x_in[0], x_in[1] + thetas_grid.get_cell_length(1));
  xi.emplace_back(x_in[0] + thetas_grid.get_cell_length(0), x_in[1] + thetas_grid.get_cell_length(1));

  std::vector<Eigen::Vector2d> loc_thetas;
  loc_thetas.push_back(thetas_grid.template unmap<Eigen::Vector2d>(xi[0][0], xi[0][1]));
  loc_thetas.push_back(thetas_grid.template unmap<Eigen::Vector2d>(xi[1][0], xi[1][1]));
  loc_thetas.push_back(thetas_grid.template unmap<Eigen::Vector2d>(xi[2][0], xi[2][1]));
  loc_thetas.push_back(thetas_grid.template unmap<Eigen::Vector2d>(xi[3][0], xi[3][1]));

  std::vector<gtsam::Key> weights;
  std::vector<gtsam::Key> thetas;
  auto ff_nm = gtsam::noiseModel::Isotropic::Sigma(1, 1e0);
  auto pf_nm = gtsam::noiseModel::Isotropic::Sigma(1, 1e0);

  gtsam::NonlinearFactorGraph graph;
  PRX_DEBUG_PRINT;
  for (int i = 0; i < 4; ++i)
  {
    if (xi[i][0] > 3 || xi[i][1] > 3)
      continue;
    const prx_symbol_t theta_i{ thetas_grid(xi[i][0], xi[i][1]).first };
    const prx_symbol_t work_space_symbol{ thetas_grid(xi[i][0], xi[i][1]).second };
    // auto work_space_symbol = symbol_factory_t::create_symbol("work_space_symbol", i);
    auto weight_symbol = symbol_factory_t::create_symbol("weight_symbol", t, i);
    thetas.push_back(theta_i);
    weights.push_back(weight_symbol);
    auto function_3factor = [=](const Eigen::Vector3d& x1, const Eigen::Vector2d& x2) {
      Eigen::Vector2d xw(x1[0], x1[1]);
      double dist = (loc_thetas[0] - xw).norm();
      dist += (loc_thetas[1] - xw).norm();
      dist += (loc_thetas[2] - xw).norm();
      dist += (loc_thetas[3] - xw).norm();
      return friction_vector_t((x2 - xw).norm() / dist);
    };
    values.insert_or_assign(weight_symbol, function_3factor(x_in, loc_thetas[i]));
    graph.add(fg::function_3factor_t<1, 3, 2>(ff_nm, weight_symbol, xt_symbol, work_space_symbol, function_3factor));
    // auto function_2factor = [=](const Eigen::Vector2d& x1) { return loc_thetas[i]; };
    // values.insert_or_assign(work_space_symbol, function_2factor(loc_thetas[i]));
    // graph.add(fg::function_2factor_t<1, 2>(ff_nm, work_space_symbol, theta_i, function_2factor));
  }
  graph.add(fg::parameter_fusion_factor_t<1>(pf_nm, thetas, weights, param_t));
  return graph;
}

template <typename grid1_t, typename grid2_t, typename log_t, typename Priors, typename Ids>
void compute_grid_error(const grid1_t& frictions_grid, const grid2_t& gt_grid, log_t& log, const int iter,
                        const Priors& priors, Ids& ids)
{
  // grid1_t error_grid{ frictions_grid };
  // // std::cout << error_grid << std::endl;
  // for (auto cell : gt_grid)
  // {
  //   auto unmap_vals = gt_grid.unmap(cell.first);
  //   const prx_symbol_t theta_simbol{ symbol_factory_t::create_symbol("param_symbol",
  //                                                                    ids(unmap_vals[0], unmap_vals[1])) };
  //   if (priors.find(theta_simbol) == priors.end())
  //   {
  //     error_grid[cell.first].array() *= 0;
  //   }
  //   else
  //   {
  //     error_grid[cell.first].array() -= cell.second;
  //   }
  // }
  // // std::cout << error_grid << std::endl;
  // using error_grid_t = decltype(*(error_grid.begin()));
  // friction_vector_t grid_error{ friction_vector_t::Zero() };
  // grid_error = std::accumulate(error_grid.begin(), error_grid.end(), grid_error,
  //                              [](auto a, error_grid_t& b) { return a + b.second.cwiseAbs(); });
  // // double sqr_error = std::accumulate(error_grid.begin(), error_grid.end(), 0.0,
  // if (grid_error.sum() > 1e100)
  // {
  //   std::cout << "error_grid: " << error_grid << std::endl;
  //   std::cout << "error: " << grid_error.transpose() << std::endl;
  // }
  // log.log(iter, grid_error.sum());
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
        Eigen::VectorXd theta{ Eigen::VectorXd::Ones(3) };
        if (results.exists(theta_simbol))
        {
          theta = results.at<Eigen::VectorXd>(theta_simbol);
        }
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
  auto params = param_loader("executables/factor_graphs/fm_basis.yaml", argc, argv);

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
  const std::string traj_real_file{ fm_out_dir + "fmbasis_analytical_fixed_goals_trajs_real.txt" };
  const std::string plan_real_file{ fm_out_dir + "fmbasis_analytical_fixed_goals_plans_real.txt" };
  const std::string traj_fg_file{ fm_out_dir + "fmbasis_analytical_fixed_goals_trajs_fg.txt" };
  const std::string fg_graph_file{ fm_out_dir + "fmbasis_factor_graph.dot" };

  std::remove(traj_real_file.c_str());
  std::remove(plan_real_file.c_str());
  std::remove(traj_fg_file.c_str());
  std::remove(fg_graph_file.c_str());

  std::ofstream ofs_gt_frmap, ofs_plans, ofs_goals, ofs_traj_real;
  ofs_goals.open(fm_out_dir + "fmbasis_analytical_fixed_goals.txt", std::ofstream::trunc);
  ofs_gt_frmap.open(fm_out_dir + "fmbasis_friction_map_fixed_goals.txt", std::ofstream::trunc);
  // friction_map_file.open(fm_out_dir + "concurrent_fg_friction_map.txt");
  fg_logger_t friction_map_logger(fm_out_dir + "fmbasis_fg_friction_map.txt", ' ', "-");

  // const std::vector<std::pair<double, double>> env_bounds{ std::make_pair(0.0, 3.0), std::make_pair(0.0, 3.0) };
  const int divisions{ 10 };
  const std::vector<std::pair<double, double>> env_bounds{ std::make_pair(0.0, x_max), std::make_pair(0.0, y_max) };
  prx::regular_grid_t<friction_vector_t, 2> frictions_grid{ env_bounds, divisions };
  prx::regular_grid_t<std::pair<prx_symbol_t, prx_symbol_t>, 2> thetas_grid{ env_bounds, divisions };

  const double initial_friction{ params["initial_friction"].as<double>() };
  const Eigen::VectorXd initial_friction_vec{ friction_vector_t::Ones() * initial_friction };

  using Container2D = std::vector<double>;
  std::function<friction_vector_t(const Container2D&)> friction_grid_initializer = [&](const Container2D&) {
    return initial_friction_vec;
  };

  gtsam::NonlinearFactorGraph thetas_workspace_graph;
  gtsam::Values thetas_workspace_values;
  auto workspace_theta_noise_model = gtsam::noiseModel::Isotropic::Sigma(1, 1e0);
  std::function<std::pair<prx_symbol_t, prx_symbol_t>(const Container2D&)> thetas_grid_initializer =
      [&](const Container2D& xy) {
        const int x{ static_cast<int>(xy[0]) };
        const int y{ static_cast<int>(xy[1]) };
        const prx_symbol_t param_symbol{ symbol_factory_t::create_symbol("param_symbol", x, y) };
        const prx_symbol_t work_space_symbol{ symbol_factory_t::create_symbol("work_space_symbol", x, y) };

        auto function_2factor = [=](const Eigen::Vector2d& x1) {
          return friction_vector_t(Eigen::Vector2d(x, y).norm());
        };
        thetas_workspace_values.insert_or_assign(work_space_symbol,
                                                 Eigen::VectorXd(function_2factor(Eigen::Vector2d::Zero())));
        thetas_workspace_values.insert_or_assign(param_symbol, initial_friction_vec);
        thetas_workspace_graph.add(fg::function_2factor_t<1, 2>(workspace_theta_noise_model, param_symbol,
                                                                work_space_symbol, function_2factor));
        return std::make_pair(param_symbol, work_space_symbol);
      };

  frictions_grid.populate_grid(friction_grid_initializer);
  thetas_grid.populate_grid(thetas_grid_initializer);
  // std::cout << "thetas_grid: " << std::endl;
  // for (auto cell : thetas_grid)
  // {
  //   std::cout << "[";
  //   for (int i = 0; i < 2; ++i)
  //   {
  //     std::cout << thetas_grid.template unmap_key<Eigen::Vector2d>(cell.first)[i];
  //     if (i < 2 - 1)
  //       std::cout << ", ";
  //   }
  //   std::cout << "]: " << prx::key_formatter(cell.second.first) << "\n";
  // }
  // exit(0);
  auto gt_grid = [&](const double x, const double y) {
    double friction = 1;
    if (y < 1.5)
    {
      friction = 2 * y / 1.5;
    }
    else
    {
      friction = 2 * (y_max - y) / 1.5;
    }
    return friction;
  };

  PRX_DEBUG_PRINT;
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

    friction_params = friction_vector_t{ gt_grid(x1, y1) };

    if (write_to_file)
    {
      ofs_gt_frmap << friction_params.transpose() << "\n";
    }
    if (x < 0.0 || x_max < x || y < 0.0 || y_max < y)
    {
      friction_params = friction_vector_t::Ones();
    }
    ps->copy_from(friction_params.tail(1));
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
    }
    // std::cout << x << ", " << y << "\tfriction_params: " << friction_params.transpose() << std::endl;
    ps->copy_from(friction_params.tail(1));
    ps->enforce_bounds();
  };

  PRX_DEBUG_PRINT;
  write_to_file = true;
  world_model.world_change_function = ground_truth_world;
  const double grid_stepping{ 1.0 / static_cast<double>(divisions) };
  for (double i = 0; i < x_max; i += 0.05)
  {
    for (double j = 0; j < y_max; j += 0.05)
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

  PRX_DEBUG_PRINT;
  trajectory_t traj_real(ss);
  trajectory_t traj_fg(ss);
  trajectory_t accum_traj(ss);
  traj_real.clear();
  traj_fg.clear();
  accum_traj.clear();
  plan_t accum_plan(cs);

  const int num_trajs{ params["num_trajs"].as<int>() };

  space_point_t goal = ss->make_point();
  condition_check_t checker(params["checker_type"].as<>(), params["checker_value"].as<int>());
  auto cc = create_default_goal_check(ss, goal, goal_region_radius);
  condition_check_t check_goal_reached(cc);
  checker.add_condition(&check_goal_reached);

  PRX_DEBUG_PRINT;
  // Factor graphs
  // gtsam::NonlinearFactorGraph graph;
  // gtsam::NonlinearFactorGraph graph_theta;
  gtsam::LevenbergMarquardtParams lm_params{ fg_params() };
  gtsam::Values init_vals;
  gtsam::Values results;

  prx::regular_grid_t<prx_symbol_t, 2> fg_grid{ env_bounds, divisions };
  int cell_number{ 0 };
  for (double x = 0; x < x_max; x += grid_stepping)
  {
    for (double y = 0; y < y_max; y += grid_stepping)
    {
      fg_grid(x, y) = cell_number;
      // frictions_grid(x, y) = friction_vector_t::Ones() * params["initial_friction"].as<double>();
      cell_number++;
    }
  }

  PRX_DEBUG_PRINT;
  std::unordered_map<prx_symbol_t, Eigen::MatrixXd> theta_priors;

  // gtsam::NonlinearFactorGraph graph_theta{ compute_thetas_graph(x_max, y_max, grid_stepping, fg_grid, frictions_grid,
  //                                                               ps, results, init_vals, friction_map_logger, -1,
  //                                                               theta_priors) };

  PRX_DEBUG_PRINT;
  const int increment = static_cast<int>(frequency / simulation_step);
  std::cout << "increment: " << increment << std::endl;
  auto x_sigma = gtsam::noiseModel::Isotropic::Sigma(ss_dim, 1e-3);
  auto dm = gtsam::noiseModel::Isotropic::Sigma(ss_dim, 1e-0);
  auto cs_dm = gtsam::noiseModel::Isotropic::Sigma(cs_dim, 1e-3);
  auto t_dm = gtsam::noiseModel::Isotropic::Sigma(1, 1e-3);
  auto p_cm = gtsam::noiseModel::Isotropic::Sigma(3, 1e0);

  PRX_DEBUG_PRINT;
  fg::formatter_t graph_formatter;

  fg_logger_t lg(out_path + "friction_maps/fg_concurrent_log.txt", ' ', "-");
  logger_t grid_error_lg(out_path + "friction_maps/fg_concurrent_error_grid_log.txt", ' ');
  int fg_iters{ 0 };
  space_point_t start_state = ss->make_point();
  const int initial_goal{ params["initial_goal"].as<int>() };
  PRX_DEBUG_PRINT;
  for (int i = initial_goal; i < num_trajs; ++i)
  {
    std::cout << "Going into trajectory: " << i << "..." << std::endl;

    compute_grid_error(frictions_grid, gt_grid, grid_error_lg, fg_iters, theta_priors, fg_grid);
    curr_freq = frequency;
    traj_real.clear();
    traj_fg.clear();
    checker.reset();
    omnibot_controller->get_plan()->clear();

    // i = initial_goal;
    ss->copy(start_state, goals[i % goals.size()]);
    n_ss->add_noise(start_state);
    // n_ss->copy(start_state, goals[i]);
    // ss->copy_from(goals[i]);
    // n_ss->copy_to(start_state);
    ss->copy(goal, goals[(i + 1) % goals.size()]);
    omnibot_controller->set_goal(goal);

    std::cout << "start_state: " << start_state << std::endl;

    // real_friction = true;
    world_model.world_change_function = ground_truth_world;
    sg->propagate(start_state, omnibot_controller, checker, traj_real);

    std::cout << "traj_real: " << traj_real.size() << std::endl;
    const plan_t plan{ *(omnibot_controller->get_plan()) };
    std::cout << "plan size: " << plan.size() << std::endl;

    gtsam::NonlinearFactorGraph trajectory_graph;
    gtsam::Values trajectory_values;

    gtsam::NonlinearFactorGraph weights_graph;
    gtsam::Values weights_values;
    // std::set<prx_symbol_t> thetas_used; 3.03112 0.676059
    PRX_DEBUG_PRINT;
    for (unsigned xi = 0; xi < traj_real.size(); xi += increment)
    {
      if (xi < traj_real.size() - increment - 1)
      {
        const Eigen::Vector3d x0{ traj_real[xi]->vector() };
        const Eigen::Vector3d x1{ traj_real[xi + increment]->vector() };
        // const double y0{ traj_real[xi]->vector()[1] };
        // const double x1{ traj_real[xi + increment]->vector()[0] };
        // const double y1{ traj_real[xi + increment]->vector()[1] };

        auto state_symbol = symbol_factory_t::create_symbol("state_symbol", i, xi);
        auto next_state_symbol = symbol_factory_t::create_symbol("state_symbol", i, xi + increment);
        auto control_symbol = symbol_factory_t::create_symbol("control_symbol", i, xi);
        auto time_symbol = symbol_factory_t::create_symbol("time_symbol", i, xi);
        auto param_symbol = symbol_factory_t::create_symbol("param_symbol_X", i, xi);

        Eigen::VectorXd t_vec{ (Eigen::VectorXd(1) << plan[xi].duration * increment).finished() };
        trajectory_graph.addPrior(state_symbol, traj_real[xi]->vector<>(), x_sigma);
        trajectory_graph.addPrior(control_symbol, plan[xi].control->vector<>(), cs_dm);
        trajectory_graph.addPrior(time_symbol, t_vec, t_dm);

        trajectory_values.insert(state_symbol, traj_real[xi]->vector<>());
        trajectory_values.insert(control_symbol, plan[xi].control->vector<>());
        trajectory_values.insert(time_symbol, t_vec);
        trajectory_values.insert(param_symbol, (Eigen::VectorXd(1) << 1).finished());

        if (xi + increment >= traj_real.size() - increment - 1)
        {
          trajectory_graph.addPrior(next_state_symbol, traj_real[xi + increment]->vector<>(), x_sigma);
          trajectory_values.insert(next_state_symbol, traj_real[xi + increment]->vector<>());
        }
        trajectory_graph.add(propagation_factor_5_t<3, 4, 3>(state_symbol, next_state_symbol, control_symbol,
                                                             time_symbol, param_symbol, dm, sg));

        weights_graph.add(find_closest_thetas(xi, state_symbol, param_symbol, x0, thetas_grid, weights_values));
        weights_graph.add(find_closest_thetas(xi, state_symbol, param_symbol, x1, thetas_grid, weights_values));
      }
    }
    PRX_DEBUG_PRINT;

    gtsam::NonlinearFactorGraph graph;
    // graph.add(graph_theta);
    graph.add(thetas_workspace_graph);
    graph.add(trajectory_graph);
    graph.add(weights_graph);
    gtsam::Values values;
    values.insert_or_assign(thetas_workspace_values);
    values.insert_or_assign(trajectory_values);
    values.insert_or_assign(weights_values);

    world_model.world_change_function = fg_world;
    std::cout << "Graph: " << graph.size() << std::endl;
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
    auto results = fg_utilities::optimize_and_log(optimizer, lm_params, lg, fg_iters);

    graph.saveGraph(fg_graph_file, results, prx::key_formatter, graph_formatter);
    // gtsam::Marginals marginals{ graph_trajs, results };
    // for (auto theta_u : thetas_used)
    // {
    //   // std::cout << prx::key_formatter(theta_u) << " Inf: " << marginals.marginalInformation(theta_u)
    //   //           << " Cov: " << marginals.marginalCovariance(theta_u) << std::endl;
    //   theta_priors[theta_u] = marginals.marginalCovariance(theta_u);
    // }
    fg_iters += optimizer.iterations();

    // graph_theta = compute_thetas_graph(x_max, y_max, grid_stepping, fg_grid, frictions_grid, ps, results, init_vals,
    //                                    friction_map_logger, fg_iters, theta_priors);

    std::cout << "init_vals size: " << init_vals.size() << std::endl;
    world_model.world_change_function = ground_truth_world;
    sg->propagate(start_state, plan, traj_fg);
    std::cout << "traj_fg: " << traj_fg.size() << std::endl;
    traj_fg.to_file(traj_fg_file, std::ofstream::app);

    accum_traj += traj_real;
    accum_plan += plan;
  }
  // for (auto pair : theta_priors)
  // {
  //   std::cout << prx::key_formatter(pair.first) << ": " << pair.second << std::endl;
  // }
  std::cout << "Finishing..." << std::endl;
  compute_grid_error(frictions_grid, gt_grid, grid_error_lg, fg_iters, theta_priors, fg_grid);

  std::cout << "fm_out_dir:" << fm_out_dir << "\n";
  std::cout << "traj_real_file:" << traj_real_file << "\n";
  std::cout << "plan_real_file:" << plan_real_file << "\n";
  std::cout << "traj_fg_file:" << traj_fg_file << "\n";
  std::cout << "fg_graph_file:" << fg_graph_file << "\n";
  // std::cout << "FG logger:" << lg.get_filename() << "\n";
  // std::cout << "grid_error_lg:" << grid_error_lg.get_filename() << "\n";

  // std::cout << "grid_error: " << sqr_error << std::endl;
  // std::cout << "grid_error: " << grid_error.transpose() << std::endl;
  // This are not "Real" trajectories/plan, is all appended into one and might
  // have discontinuities. Only used for dumping into a file.
  accum_traj.to_file(traj_real_file, std::ofstream::app);
  accum_plan.to_file(plan_real_file, std::ofstream::app);
}