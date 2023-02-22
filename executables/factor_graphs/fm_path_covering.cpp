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
#include "prx/factor_graphs/factors/friction_fusion_factor.hpp"
#include "prx/factor_graphs/factors/parameter_fusion_factor.hpp"

#include "prx/factor_graphs/utilities/utilities_functions.hpp"
#include "prx/factor_graphs/utilities/formatter.hpp"

#include <gtsam/nonlinear/Marginals.h>

namespace fs = std::filesystem;
using namespace prx;

const int X_DIM{ 3 };
const int U_DIM{ 2 };
const int TH_DIM{ 1 };
const int GRID_DIVISIONS{ 10 };
const int BASIS_DIM{ (GRID_DIVISIONS + 1) * (GRID_DIVISIONS + 1) };

const double x_max{ 3.0 };
const double y_max{ 3.0 };

using state_t = Eigen::Vector<double, X_DIM>;
using friction_vector_t = Eigen::Vector<double, TH_DIM>;
using basis_vector_t = Eigen::Vector<double, BASIS_DIM>;
// using friction_factor_t = fg::friction_fusion_factor_t<TH_DIM, BASIS_DIM>;

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
  return lm_params;
}

template <typename ThetasGrid>
basis_vector_t get_basis_vector(ThetasGrid& thetas_grid)
{
  basis_vector_t bv{ basis_vector_t::Zero() };
  std::size_t i = 0;
  for (auto pair_ : thetas_grid)
  {
    bv[i] = pair_.second[0];
    i++;
    // th[i] = values.at<theta_t>(keys_[i])[0];  // Assuming THETA_DIM==1 for now.
  }
  return bv;
}

double compute_weight(const state_t& theta_t_pos, const state_t& theta_i_pos, const double& length)
{
  const double Bx{ theta_i_pos[0] };
  const double By{ theta_i_pos[1] };

  const double Xx{ theta_t_pos[0] };
  const double Xy{ theta_t_pos[1] };

  const double delta_x{ std::fabs(Bx - Xx) };
  const double delta_y{ std::fabs(By - Xy) };

  const double D{ std::sqrt(length * length + length * length) };
  // const double D{ length };
  return std::max(1.0 - ((delta_x + delta_y) / D), 0.0);
}

template <typename ThetaPosGrid>
basis_vector_t compute_weights_vector(ThetaPosGrid& pos_grid, const state_t& theta_i_pos)
{
  basis_vector_t weights{ basis_vector_t::Zero() };
  const double cell_length{ pos_grid.get_cell_length(0) };
  std::size_t i = 0;
  for (auto pair_ : pos_grid)
  {
    const state_t pos_theta{ pos_grid.template unmap_key<state_t>(pair_.first) };
    weights[i] = compute_weight(pos_theta, theta_i_pos, cell_length);
    i++;
  }
  weights = weights / weights.sum();
  return weights;
}

template <typename ThetaPosGrid>
friction_vector_t friction_at(const double x, const double y, const basis_vector_t& thetas, ThetaPosGrid& pos_grid)
{
  const basis_vector_t weights{ compute_weights_vector(pos_grid, state_t(x, y, 0)) };
  const friction_vector_t friction_vector{ weights.adjoint() * thetas };

  // PRX_DEBUG_VAR_1(thetas.transpose());
  // PRX_DEBUG_VAR_1(weights.transpose());
  // PRX_DEBUG_VAR_1(friction_vector.transpose());

  return friction_vector;
}

template <typename ThetaPosGrid>
friction_vector_t friction_2_at(const double x, const double y, const basis_vector_t& thetas, ThetaPosGrid& pos_grid)
{
  const double min_val{ 0.5 };
  const double max_val{ 1.5 };
  // const double x_step{ x_max / static_cast<double>(GRID_DIVISIONS) };
  // const double y_step{ y_max / static_cast<double>(GRID_DIVISIONS) };
  std::vector<double> vals{ linspace(min_val, max_val, 1 + 4 / 2) };
  vals.insert(vals.end(), vals.rbegin() + 1, vals.rend());
  double per = y / y_max;
  if (y < 1.5)
  {
    per = 2 * y / 1.5;
  }
  else
  {
    per = 2 * (y_max - y) / 1.5;
  }
  return friction_vector_t(per);
}

template <typename ThetaPosGrid>
friction_vector_t friction_3_at(const double x, const double y, const basis_vector_t& thetas, ThetaPosGrid& pos_grid)
{
  const double min_val{ 0.5 };
  const double max_val{ 1.5 };
  // const double x_step{ x_max / static_cast<double>(GRID_DIVISIONS) };
  // const double y_step{ y_max / static_cast<double>(GRID_DIVISIONS) };
  std::vector<double> vals{ linspace(min_val, max_val, 1 + 4 / 2) };
  vals.insert(vals.end(), vals.rbegin() + 1, vals.rend());
  double per = y / y_max;
  return friction_vector_t(vals[static_cast<int>(per * vals.size())]);
}

template <typename ThetasGrid, typename F>
void compute_friction_map(ThetasGrid& grid, logger_t& logger, F& f)
{
  const double grid_stepping{ 1.0 / static_cast<double>(GRID_DIVISIONS) };
  const basis_vector_t basis{ get_basis_vector(grid) };
  for (double x = 0; x < x_max; x += 0.02)
  {
    for (double y = 0; y < y_max; y += 0.02)
    {
      // friction_params = friction_at(x, y, real_thetas_vector, frictions_grid);
      const friction_vector_t friction_params{ f(x, y, basis, grid) };
      logger.log(x, y, friction_params.transpose());

      // ps->copy_from(friction_params);
    }
  }
}

template <typename ThetaFrictionGrid>
void update_friction_grid(ThetaFrictionGrid& tf_grid, basis_vector_t new_frictions)
{
  std::size_t i{ 0 };
  for (auto pair_ : tf_grid)
  {
    tf_grid[pair_.first][0] = new_frictions[i];
    i++;
  }
}

template <typename FrictionGrid>
void compute_grid_error(FrictionGrid& gt_grid, FrictionGrid& fg_grid, logger_t& error_log, logger_t& thetas_error_log,
                        logger_t& interpolated_error_log, const int iter)
{
  double error{ 0.0 };

  const basis_vector_t gt_basis{ get_basis_vector(gt_grid) };
  const basis_vector_t fg_basis{ get_basis_vector(fg_grid) };

  for (auto pair_ : gt_grid)
  {
    const state_t pos_theta{ gt_grid.template unmap_key<state_t>(pair_.first) };
    const double x1{ pos_theta[0] };
    const double y1{ pos_theta[1] };
    const friction_vector_t gt_friction_params{ friction_2_at(x1, y1, gt_basis, gt_grid) };
    const friction_vector_t ft_friction_params{ friction_at(x1, y1, fg_basis, fg_grid) };
    const friction_vector_t fp_error{ gt_friction_params - ft_friction_params };

    error += std::fabs(gt_grid[pair_.first][0] - fg_grid[pair_.first][0]);

    thetas_error_log.log(iter, x1, y1, fp_error.transpose());

    // PRX_DEBUG_VAR_1(pos_theta.transpose());
  }
  error_log.log(iter, error);
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
  auto params = param_loader("executables/factor_graphs/fm_path_covering.yaml", argc, argv);

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

  ss->set_bounds({ 0.0, 0.0, -M_PI }, { x_max, y_max, M_PI });

  auto cs_lb = params["/plant/control_space_lower_bound"].as<std::vector<double>>();
  auto cs_up = params["/plant/control_space_upper_bound"].as<std::vector<double>>();
  cs->set_bounds(cs_lb, cs_up);

  const std::string fm_out_dir = out_path + "friction_maps/";
  std::unordered_map<std::string, std::string> files;
  files["traj_real_file"] = fm_out_dir + "fmbasis_trajs_real.txt";
  files["plan_real_file"] = fm_out_dir + "fmbasis_plans_real.txt";
  files["traj_fg_file"] = fm_out_dir + "fmbasis_trajs_fg.txt";
  files["fg_graph_file"] = fm_out_dir + "fmbasis_factor_graph.dot";
  files["thx_file"] = fm_out_dir + "fmbasis_factor_graph_thx.txt";
  files["idd_friction_map"] = fm_out_dir + "fmbasis_idd_friction_map.txt";
  files["goals"] = fm_out_dir + "fmbasis_goals.txt";
  files["gt_friction_map"] = fm_out_dir + "fmbasis_gt_friction_map.txt";
  files["fg_log"] = fm_out_dir + "fmbasis_fg.log";
  files["error_grid"] = fm_out_dir + "fmbasis_error_grid.log";
  files["thetas_error_log"] = fm_out_dir + "thetas_error.log";
  files["interpolated_error_log"] = fm_out_dir + "interpolated_error.log";
  // files["fg_log"] = fm_out_dir + "fg_concurrent.log";

  for (auto f : files)
  {
    std::remove(files[f.first].c_str());
  }

  std::ofstream ofs_gt_frmap, ofs_plans, ofs_goals, ofs_traj_real, ofs_thx;
  ofs_goals.open(files["goals"], std::ofstream::trunc);
  // ofs_gt_frmap.open(files["gt_friction_map"], std::ofstream::trunc);
  // fg_logger_t friction_map_logger(files["fg_log"], ' ', "-");

  logger_t logger_thx(files["thx_file"]);
  logger_t logger_idd_friction_map(files["idd_friction_map"]);
  fg_logger_t friction_map_logger(files["fg_log"], ' ', "-");
  logger_t grid_error_lg(files["error_grid"], ' ');
  logger_t thetas_error_log(files["thetas_error_log"], ' ');
  logger_t interpolated_error_log(files["interpolated_error_log"], ' ');
  logger_t gt_friction_map_log(files["gt_friction_map"], ' ');

  // const std::vector<std::pair<double, double>> env_bounds{ std::make_pair(0.0, 3.0), std::make_pair(0.0, 3.0) };
  const std::vector<std::pair<double, double>> env_bounds{ std::make_pair(0.0, x_max), std::make_pair(0.0, y_max) };
  prx::regular_grid_t<friction_vector_t, 2> frictions_grid{ env_bounds, GRID_DIVISIONS };
  prx::regular_grid_t<friction_vector_t, 2> ground_truth_thetas_grid{ env_bounds, GRID_DIVISIONS };
  const double initial_friction{ params["initial_friction"].as<double>() };
  const Eigen::VectorXd initial_friction_vec{ friction_vector_t::Ones() * initial_friction };

  using Container2D = std::vector<double>;
  std::function<friction_vector_t(const Container2D&)> friction_grid_initializer = [&](const Container2D&) {
    return initial_friction_vec;
  };

  gtsam::NonlinearFactorGraph thetas_graph;
  gtsam::Values thetas_values;

  std::function<friction_vector_t(const Container2D&)> gt_thetas_grid_initializer = [&](const Container2D& xy) {
    const double x{ xy[0] };
    const double y{ xy[1] };
    const double th{ 2 - std::fabs(-x + x_max * .5) / x_max - std::fabs(-y + y_max * .5) / y_max };
    return friction_vector_t(th);
  };

  frictions_grid.populate_grid(friction_grid_initializer);
  ground_truth_thetas_grid.populate_grid(gt_thetas_grid_initializer);

  auto vec_formatter = [](const friction_vector_t& f) { return f.transpose(); };

  files["gt_thetas_grid"] = fm_out_dir + "/gt_thetas_grid.txt";
  ground_truth_thetas_grid.to_file(files["gt_thetas_grid"], vec_formatter);

  basis_vector_t real_thetas_vector{ get_basis_vector(ground_truth_thetas_grid) };
  basis_vector_t recovered_thetas_vector{ basis_vector_t::Zero() };

  bool write_to_file = false;
  Eigen::Vector4d fg_friction_maps;
  friction_vector_t friction_params{ friction_vector_t::Zero() };
  std::function<void()> real_world = [&]()  // no-lint
  {
    const double x{ ss->at(0) };
    const double y{ ss->at(1) };
    const double th{ ss->at(2) };

    friction_params = friction_2_at(x, y, real_thetas_vector, frictions_grid);
    if (x < 0.0 || x_max < x || y < 0.0 || y_max < y)
    {
      friction_params = friction_vector_t::Ones();
    }
    if (write_to_file)
    {
      // ofs_gt_frmap << x << " " << y << " " << friction_params.transpose() << std::endl;
    }
    ps->copy_from(friction_params);
  };
  std::function<void()> fg_sim_world = [&]() {};
  std::function<void()> recovered_world = [&]()  // no-lint
  {
    const double x{ ss->at(0) };
    const double y{ ss->at(1) };
    friction_params = friction_at(x, y, recovered_thetas_vector, frictions_grid);
    if (x < 0.0 || x_max < x || y < 0.0 || y_max < y)
    {
      friction_params = friction_vector_t::Ones();
    }
    if (write_to_file)
    {
      // logger_idd_friction_map.log(x, y, friction_params.transpose());
    }
    ps->copy_from(friction_params);
    ps->enforce_bounds();
  };

  // write_to_file = true;
  world_model.world_change_function = real_world;
  compute_friction_map(ground_truth_thetas_grid, gt_friction_map_log,
                       friction_2_at<decltype(ground_truth_thetas_grid)>);
  write_to_file = false;

  std::vector<std::vector<double>> goals = {};

  const int num_trajs{ params["num_trajs"].as<int>() };
  const int num_goals{ params["num_goals"].as<int>() };

  const std::vector<std::vector<double>> input_goals{ params["goals"].as<std::vector<std::vector<double>>>() };

  for (auto v : input_goals)
  {
    goals.push_back(v);
  }

  for (int i = 0; i < num_goals - input_goals.size(); ++i)
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

  PRX_DEBUG_PRINT;
  std::unordered_map<prx_symbol_t, Eigen::MatrixXd> theta_priors;

  const int increment = static_cast<int>(frequency / simulation_step);
  std::cout << "increment: " << increment << std::endl;
  auto x_sigma = gtsam::noiseModel::Isotropic::Sigma(ss_dim, 1e-3);
  auto dm = gtsam::noiseModel::Isotropic::Sigma(ss_dim, 1e-0);
  auto cs_dm = gtsam::noiseModel::Isotropic::Sigma(cs_dim, 1e-3);
  auto t_dm = gtsam::noiseModel::Isotropic::Sigma(1, 1e-3);
  auto p_cm = gtsam::noiseModel::Isotropic::Sigma(3, 1e0);
  auto ff_nm = gtsam::noiseModel::Isotropic::Sigma(1, 1e0);
  auto weight_nm = gtsam::noiseModel::Isotropic::Sigma(BASIS_DIM, 1e-3);
  gtsam::SharedGaussian basis_nm = gtsam::noiseModel::Isotropic::Sigma(BASIS_DIM, 1e1);

  fg::formatter_t graph_formatter;

  int fg_iters{ 0 };
  space_point_t start_state = ss->make_point();
  const int initial_goal{ params["initial_goal"].as<int>() };

  auto last_state = ss->make_point();
  ss->copy(last_state, goals[0]);
  for (int i = 0; i < num_trajs; ++i)
  {
    std::cout << "Going into trajectory: " << i << "..." << std::endl;

    compute_grid_error(ground_truth_thetas_grid, frictions_grid, grid_error_lg, thetas_error_log,
                       interpolated_error_log, fg_iters);
    curr_freq = frequency;
    traj_real.clear();
    traj_fg.clear();
    checker.reset();
    omnibot_controller->get_plan()->clear();

    ss->copy(start_state, last_state);
    // n_ss->add_noise(start_state);
    ss->copy(goal, goals[(i + 1) % goals.size()]);
    omnibot_controller->set_goal(goal);

    std::cout << "start_state: " << start_state << std::endl;
    std::cout << "goal: " << goal << std::endl;

    // real_friction = true;
    world_model.world_change_function = real_world;
    sg->propagate(start_state, omnibot_controller, checker, traj_real);

    ss->copy(last_state, traj_real.back());
    traj_real.to_file(files["traj_real_file"], std::ofstream::app);
    std::cout << "traj_real: " << traj_real.size() << std::endl;
    const plan_t plan{ *(omnibot_controller->get_plan()) };
    std::cout << "plan size: " << plan.size() << std::endl;

    gtsam::NonlinearFactorGraph trajectory_graph;
    gtsam::Values trajectory_values;

    gtsam::NonlinearFactorGraph weights_graph;
    gtsam::Values weights_values;
    std::unordered_map<prx_symbol_t, prx_symbol_t> thetas_used;  // 3.03112 0.676059
                                                                 // PRX_DEBUG_PRINT;
    auto param_symbol_basis = symbol_factory_t::create_symbol("param_symbol", 0, 0);
    basis_vector_t big_vector = get_basis_vector(frictions_grid);
    weights_values.insert(param_symbol_basis, big_vector);
    weights_graph.addPrior(param_symbol_basis, big_vector, basis_nm);
    for (unsigned xi = 0; xi < traj_real.size(); xi += increment)
    {
      if (xi + increment < traj_real.size())
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
        auto weights_symbol = symbol_factory_t::create_symbol("weight_symbol", i, xi);

        thetas_used[param_symbol] = state_symbol;

        Eigen::VectorXd t_vec{ (Eigen::VectorXd(1) << plan[xi].duration * increment).finished() };
        trajectory_graph.addPrior(state_symbol, traj_real[xi]->vector<>(), x_sigma);
        trajectory_graph.addPrior(control_symbol, plan[xi].control->vector<>(), cs_dm);
        trajectory_graph.addPrior(time_symbol, t_vec, t_dm);

        trajectory_values.insert(state_symbol, traj_real[xi]->vector<>());
        trajectory_values.insert(control_symbol, plan[xi].control->vector<>());
        trajectory_values.insert(time_symbol, t_vec);
        trajectory_values.insert(param_symbol, (Eigen::VectorXd(1) << 1).finished());

        const basis_vector_t weight_i{ compute_weights_vector(frictions_grid, traj_real[xi]->vector<state_t>()) };
        weights_values.insert(weights_symbol, weight_i);
        weights_graph.addPrior(weights_symbol, weight_i, weight_nm);
        if (xi + increment >= traj_real.size() - increment - 1)
        {
          trajectory_graph.addPrior(next_state_symbol, traj_real[xi + increment]->vector<>(), x_sigma);
          trajectory_values.insert(next_state_symbol, traj_real[xi + increment]->vector<>());
        }
        trajectory_graph.add(propagation_factor_5_t<3, 4, 1>(state_symbol, next_state_symbol, control_symbol,
                                                             time_symbol, param_symbol, dm, sg));
        weights_graph.add(
            fg::friction_fusion_factor_t<TH_DIM, BASIS_DIM>(ff_nm, param_symbol_basis, weights_symbol, param_symbol));
      }
    }

    gtsam::NonlinearFactorGraph graph;
    graph.add(trajectory_graph);
    graph.add(weights_graph);
    gtsam::Values values;
    values.insert_or_assign(thetas_values);
    values.insert_or_assign(trajectory_values);
    values.insert_or_assign(weights_values);

    world_model.world_change_function = fg_sim_world;
    std::cout << "Graph: " << graph.size() << std::endl;
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
    results = fg::utilities::optimize_and_log(optimizer, lm_params, friction_map_logger, fg_iters);

    graph.saveGraph(files["fg_graph_file"], results, prx::key_formatter, graph_formatter);

    const basis_vector_t new_frictions{ results.at<basis_vector_t>(param_symbol_basis) };
    update_friction_grid(frictions_grid, new_frictions);
    PRX_DEBUG_VAR_1(new_frictions.transpose());

    files["updated_frictions"] = fm_out_dir + "updated_frictions.txt";
    frictions_grid.to_file(files["updated_frictions"]);

    gtsam::Marginals marginals{ graph, results };
    basis_nm->Covariance(marginals.marginalCovariance(param_symbol_basis));
    for (auto pair_th_x : thetas_used)
    {
      auto th = results.at<Eigen::VectorXd>(pair_th_x.first);
      auto x = results.at<Eigen::VectorXd>(pair_th_x.second);
      logger_thx.log(x[0], x[1], th[0]);
    }

    fg_iters += optimizer.iterations();

    std::cout << "init_vals size: " << init_vals.size() << std::endl;
    world_model.world_change_function = real_world;
    sg->propagate(start_state, plan, traj_fg);
    std::cout << "traj_fg: " << traj_fg.size() << std::endl;
    traj_fg.to_file(files["traj_fg_file"], std::ofstream::app);

    accum_plan += plan;
  }
  // results.print("Results: ", prx::key_formatter);

  compute_grid_error(ground_truth_thetas_grid, frictions_grid, grid_error_lg, thetas_error_log, interpolated_error_log,
                     fg_iters);
  world_model.world_change_function = recovered_world;
  // write_to_file = true;
  recovered_thetas_vector = get_basis_vector(frictions_grid);
  compute_friction_map(frictions_grid, logger_idd_friction_map, friction_at<decltype(frictions_grid)>);

  // write_to_file = false;

  std::cout << "Finishing...\nFiles:" << std::endl;

  for (auto f : files)
  {
    std::cout << f.first << ": " << f.second << std::endl;
  }

  // This are not "Real" trajectories/plan, is all appended into one and might
  // have discontinuities. Only used for dumping into a file.
  // accum_traj.to_file(files["traj_real_file"], std::ofstream::app);
  accum_plan.to_file(files["plan_real_file"], std::ofstream::app);
}