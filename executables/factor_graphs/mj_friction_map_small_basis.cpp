#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_set>
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
#include "prx/utilities/general/constants.hpp"
#include "prx/utilities/general/range.hpp"
#include "prx/utilities/geometry/regular_grid.hpp"
#include "prx/utilities/math/first_order_derivative.hpp"

#include "prx/visualization/three_js_group.hpp"

#include "prx/factor_graphs/defs.hpp"
#include "prx/factor_graphs/factors/friction_fusion_factor.hpp"
#include "prx/factor_graphs/factors/mj_friction_propagation.hpp"
#include "prx/factor_graphs/factors/parameter_fusion_factor.hpp"
#include "prx/factor_graphs/factors/positive_vector_factor.hpp"
#include "prx/factor_graphs/factors/state_prior.hpp"
#include "prx/factor_graphs/utilities/constants.hpp"
#include "prx/factor_graphs/utilities/fg_logger.hpp"
#include "prx/factor_graphs/utilities/friction_map.hpp"
#include "prx/factor_graphs/utilities/utilities_functions.hpp"

#include "prx/mujoco/mj_simulator.hpp"
#include "prx/mujoco/plants/mj_friction_plant.hpp"
#include <gtsam/nonlinear/Marginals.h>

#include "friction_maps.hpp"

using namespace prx;

const int XMAX{ 20 };
const int YMAX{ 20 };
const int GRID_DIVISIONS{ 10 };
const int BASIS_DIM{ (GRID_DIVISIONS + 1) * (GRID_DIVISIONS + 1) };

const Eigen::Index TH_DIM{ 1 };

using friction_vector_t = Eigen::Vector<double, TH_DIM>;
using basis_vector_t = Eigen::Vector<double, BASIS_DIM>;

std::string to_zero_lead(const int value, const unsigned precision)
{
  std::ostringstream oss;
  oss << std::setw(precision) << std::setfill('0') << value;
  return oss.str();
}

int main(int argc, char** argv)
{
  const std::string params_file{ "executables/factor_graphs/mj_friction_map.yaml" };
  param_loader params(params_file, argc, argv);

  init_random(params["random_seed"].as<int>());

  std::string model_filename = params["model_file"].as<std::string>();

  const bool visualize{ params["visualize"].as<bool>() };
  std::shared_ptr<mujoco_simulator_t> sim = std::make_shared<mujoco_simulator_t>(model_filename, visualize);

  std::shared_ptr<prx::mujoco_plant_t> mj_plant = std::make_shared<prx::mujoco_plant_t>("mujoco_plant");
  // system_ptr_t system;
  //   system.reset(new mujoco_plant_t("mujoco_plant"));
  // auto mj_ptr = std::dynamic_pointer_cast<mujoco_plant_t>(system);
  // auto sim_ptr = std::static_pointer_cast<mujoco_simulator_t>(this->shared_ptr());
  mj_plant->initialize(sim);
  prx::mujoco::mj_friction_plant_t::add_friction_to_plant(sim, mj_plant);

  sim->init_simulator(mj_plant);

  auto context = sim->get_context("mujoco");
  const auto sg = context.first;
  const auto ss = sg->get_state_space();
  const auto cs = sg->get_control_space();
  const auto ps = sg->get_parameter_space();
  prx_assert(ss != nullptr, "State space is null!!!");
  prx_assert(cs != nullptr, "Control space is null!!!");
  prx_assert(ps != nullptr, "Parameter space is null!!!");
  // const std::size_t ss_dim{ 27 };
  const std::size_t ss_dim{ ss->get_dimension() };
  const std::size_t cs_dim{ cs->get_dimension() };
  const auto ps_dim = ps->get_dimension();
  // const std::size_t ps_dim{ 1 };
  // PRX_DEBUG_VAR_1(ps_dim);

  // using PropagationMj = prx::fg::mj_friction_propagation_t<Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic>;
  using PropagationMj = prx::fg::mj_friction_estimation_t<Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic>;

  using State = typename PropagationMj::State;
  using Control = typename PropagationMj::Control;
  // using Time = typename PropagationMj::Time;
  using Theta = typename PropagationMj::Theta;
  using MjFunction = typename PropagationMj::MjFunction;
  using Position = Eigen::Vector<double, 2>;
  using BasisPosition = Eigen::Matrix<double, 4, 2>;

  std::unordered_map<std::string, logger_t> logs{};
  friction_map::init_logmap(logs, "small_");

  const std::vector<std::pair<double, double>> env_bounds{ std::make_pair(-XMAX, XMAX), std::make_pair(-YMAX, YMAX) };
  prx::regular_grid_t<friction_vector_t, 2> frictions_grid{ env_bounds, GRID_DIVISIONS };
  prx::regular_grid_t<friction_vector_t, 2> visited_grid{ env_bounds, GRID_DIVISIONS };
  prx::regular_grid_t<prx::prx_symbol_t, 2> basis_grid{ env_bounds, GRID_DIVISIONS };
  const double initial_friction{ params["initial_friction"].as<double>() };
  const Eigen::VectorXd initial_friction_vec{ friction_vector_t::Ones(ps_dim) * initial_friction };

  std::size_t basis_idx{ 0 };
  using Container2D = std::vector<double>;
  std::function<prx::prx_symbol_t(const Container2D&)> basis_grid_initializer = [&](const Container2D&) {
    const prx::prx_symbol_t basis_symbol{ symbol_factory_t::create_hashed_symbol("basis", basis_idx) };
    basis_idx++;
    return basis_symbol;
  };
  basis_grid.populate_grid(basis_grid_initializer);
  frictions_grid.populate_grid(initial_friction_vec);
  visited_grid.populate_grid(friction_vector_t::Zero());

  int floor_id{ 0 };
  int total_geoms{ sim->_mj_model->ngeom };
  for (int i = 0; i < total_geoms; ++i)
  {
    std::string g1 = std::string(sim->_mj_model->names + sim->_mj_model->name_geomadr[i]);
    if (g1 == "floor0")
      floor_id = i;
  }
  MjFunction fg_mjfn = [&](const State& x0, const State& x1, const Control& u, const Theta& th)  // no-lint
  {
    Theta th_transform(Theta::Zero(ps_dim));
    th_transform[0] = std::exp(-th[0]);

    ps->copy_from(th_transform);
  };

  gtsam::LevenbergMarquardtParams lm_params{ fg::utilities::default_levenberg_marquardt_parameters() };
  lm_params.setUseFixedLambdaFactor(true);
  lm_params.setMaxIterations(2000);
  lm_params.setRelativeErrorTol(1e-10);
  lm_params.setAbsoluteErrorTol(1e-10);
  lm_params.setlambdaFactor(1);

  friction_map._noise_models["positive_friction"] = gtsam::noiseModel::Isotropic::Sigma(1, 1e-5);
  friction_map._noise_models["parameter_space"] = gtsam::noiseModel::Isotropic::Sigma(1, 1e0);
  friction_map._noise_models["small_basis"] = gtsam::noiseModel::Isotropic::Sigma(1, 1e-5);

  // noise_models["control_space"] = gtsam::noiseModel::Isotropic::Sigma(cs_dim, 1e-3);
  // noise_models["basis"] = gtsam::noiseModel::Isotropic::Sigma(BASIS_DIM, 0);
  // noise_models["propagation"] = gtsam::noiseModel::Isotropic::Sigma(ss_dim, 1e-0);
  // noise_models["friction_fusion"] = gtsam::noiseModel::Isotropic::Sigma(1, 1e0);
  // // noise_models["weight"] = gtsam::noiseModel::Isotropic::Sigma(BASIS_DIM, 1e0);
  // noise_models["positive_basis"] = gtsam::noiseModel::Isotropic::Sigma(BASIS_DIM, 1e-5);
  // noise_models["guard"] = gtsam::noiseModel::Isotropic::Sigma(BASIS_DIM, 1e-5);
  // gtsam::SharedGaussian basis_nm = gtsam::noiseModel::Isotropic::Sigma(BASIS_DIM, 1e1);

  std::size_t total_plan_trajectories_files{ params["total_plan_traj_files_to_use"].as<std::size_t>() };
  std::string data_path{ params["data_path"].as<std::string>() };

  plan_t plan(cs);
  trajectory_t trajectory(ss);

  const double length_0{ 4 };
  const double length_1{ 4 };

  std::size_t fg_iterations{ 0 };
  std::unordered_map<prx::prx_symbol_t, Position> symbol_positions;
  std::unordered_map<prx::prx_symbol_t, gtsam::SharedGaussian> basis_covariances;
  gtsam::Values resulting_values;
  gtsam::NonlinearFactorGraph accum_fg;
  for (std::size_t idx = 0; idx < total_plan_trajectories_files; ++idx)
  {
    const std::string traj_path = data_path + "/traj_" + to_zero_lead(idx, 5) + ".txt";
    const std::string plan_path = data_path + "/plan_" + to_zero_lead(idx, 5) + ".txt";
    PRX_DEBUG_VAR_1(traj_path);
    PRX_DEBUG_VAR_1(plan_path);
    plan.clear();
    trajectory.clear();
    plan.from_file(plan_path);
    trajectory.from_file(traj_path);
    plan.expand();
    PRX_DEBUG_VAR_2(plan.size(), trajectory.size());
    if (plan.size() + 1 != trajectory.size())
    {
      prx_warn("Plan/Traj " << idx << "mismatch on sizes");
      continue;
    }
    friction_map::plan_trajectory_t plan_trajectory(&plan, &trajectory);
    gtsam::NonlinearFactorGraph trajectory_graph;
    gtsam::Values trajectory_values;

    gtsam::NonlinearFactorGraph weights_graph;
    gtsam::Values weights_values;
    std::unordered_map<prx_symbol_t, prx_symbol_t> thetas_used;

    const prx_symbol_t param_symbol_basis{ symbol_factory_t::create_hashed_symbol("param_basis", 0) };
    const prx_symbol_t guard_symbol{ symbol_factory_t::create_hashed_symbol("guard", 0) };
    // basis_vector_t big_vector = friction_map::basis_vector_from_grid<basis_vector_t>(frictions_grid);
    // weights_values.insert(param_symbol_basis, big_vector);
    // weights_graph.addPrior(param_symbol_basis, big_vector, noise_models["basis"]);
    // weights_graph.add(fg::positive_vector_factor_t<BASIS_DIM>(param_symbol_basis, noise_models["positive_basis"]));

    std::size_t step_i{ 0 };
    std::unordered_set<prx::prx_symbol_t> basis_used;
    for (auto state_ctrl_tuple : plan_trajectory)
    {
      const prx::space_point_t xi_pt = std::get<0>(state_ctrl_tuple);
      const prx::plan_step_t ui_pt = std::get<1>(state_ctrl_tuple);
      const prx::space_point_t xip1_pt = std::get<2>(state_ctrl_tuple);

      const State xi_v{ xi_pt->vector() };
      const Control ui_v{ ui_pt.control->vector() };
      const Eigen::VectorXd ti_v{ (Eigen::VectorXd(1) << ui_pt.duration).finished() };
      const State xip1_v{ xip1_pt->vector() };

      prx_symbol_t state_symbol{ symbol_factory_t::create_hashed_symbol("state_symbol", idx, step_i) };
      prx_symbol_t next_state_symbol{ symbol_factory_t::create_hashed_symbol("state_symbol", idx, step_i + 1) };
      prx_symbol_t control_symbol{ symbol_factory_t::create_hashed_symbol("control_symbol", idx, step_i) };
      prx_symbol_t param_symbol{ symbol_factory_t::create_hashed_symbol("param_symbol", idx, step_i) };
      // prx_symbol_t weights_symbol{ symbol_factory_t::create_hashed_symbol("weight_symbol", idx, step_i) };

      symbol_positions[param_symbol] = xi_v.head(2);

      visited_grid(xi_v[0], xi_v[1])[0] = 1;
      // auto uk = visited_grid.unmap<std::array<double, 2>>(xi_v[0], xi_v[1]);
      trajectory_values.insert(param_symbol, (Eigen::VectorXd(1) << initial_friction).finished());

      // const basis_vector_t weight_i{ friction_map::weights_vector_given_state<basis_vector_t>(frictions_grid, xi_v)
      // }; weights_used.push_back(weights_symbol); weights_values.insert(weights_symbol, weight_i);
      // weights_graph.addPrior(weights_symbol, weight_i, noise_models["weight"]);

      trajectory_graph.add(fg::positive_vector_factor_t<TH_DIM>(param_symbol, noise_models["positive_friction"]));
      trajectory_graph.add(PropagationMj(param_symbol, noise_models["parameter_space"], sg, fg_mjfn, ss_dim, cs_dim,
                                         ps_dim, xi_pt, xip1_pt, ui_pt.control, prx::simulation_step));

      // weights_graph.add(fg::friction_fusion_factor_t<TH_DIM, BASIS_DIM, Eigen::Dynamic, decltype(frictions_grid)>(
      //     noise_models["friction_fusion"], param_symbol_basis, param_symbol, guard_symbol, xi_v, &frictions_grid));

      // Eigen::Vector2d position{ _frictions_grid->template unmap_key<Eigen::Vector2d>((*iter).first) };

      Position position_0{ xi_v.head(2) };
      Position position_1{ xi_v.head(2) + Position(0, length_1) };
      Position position_2{ xi_v.head(2) + Position(length_0, 0) };
      Position position_3{ xi_v.head(2) + Position(length_0, length_1) };

      const prx::prx_symbol_t basis_symbol_0{ basis_grid(position_0[0], position_0[1]) };
      const prx::prx_symbol_t basis_symbol_1{ basis_grid(position_1[0], position_1[1]) };
      const prx::prx_symbol_t basis_symbol_2{ basis_grid(position_2[0], position_2[1]) };
      const prx::prx_symbol_t basis_symbol_3{ basis_grid(position_3[0], position_3[1]) };
      BasisPosition basis_positions;
      basis_positions.row(0) = frictions_grid.unmap<Eigen::Vector2d>(position_0[0], position_0[1]);
      basis_positions.row(1) = frictions_grid.unmap<Eigen::Vector2d>(position_1[0], position_1[1]);
      basis_positions.row(2) = frictions_grid.unmap<Eigen::Vector2d>(position_2[0], position_2[1]);
      basis_positions.row(3) = frictions_grid.unmap<Eigen::Vector2d>(position_3[0], position_3[1]);
      symbol_positions[basis_symbol_0] = basis_positions.row(0);
      symbol_positions[basis_symbol_1] = basis_positions.row(1);
      symbol_positions[basis_symbol_2] = basis_positions.row(2);
      symbol_positions[basis_symbol_3] = basis_positions.row(3);

      // weights_graph.add(fg::positive_vector_factor_t<TH_DIM>(basis_symbol_0, noise_models["positive_friction"]));
      // weights_graph.add(fg::positive_vector_factor_t<TH_DIM>(basis_symbol_1, noise_models["positive_friction"]));
      // weights_graph.add(fg::positive_vector_factor_t<TH_DIM>(basis_symbol_2, noise_models["positive_friction"]));
      // weights_graph.add(fg::positive_vector_factor_t<TH_DIM>(basis_symbol_3, noise_models["positive_friction"]));
      weights_graph.add(fg::friction_local_fusion_factor_t<TH_DIM, State, BasisPosition>(
          noise_models["small_basis"], param_symbol, basis_symbol_0, basis_symbol_1, basis_symbol_2, basis_symbol_3,
          xi_v, basis_positions, length_0, TH_DIM, 1));
      const Eigen::Vector<double, 1> one(1);
      weights_values.insert_or_assign(basis_symbol_0, one);
      weights_values.insert_or_assign(basis_symbol_1, one);
      weights_values.insert_or_assign(basis_symbol_2, one);
      weights_values.insert_or_assign(basis_symbol_3, one);

      step_i++;
    }

    for (auto basis_symbol : basis_used)
    {
      weights_graph.add(fg::positive_vector_factor_t<TH_DIM>(basis_symbol, noise_models["positive_friction"]));
    }

    gtsam::NonlinearFactorGraph graph;
    graph.add(trajectory_graph);
    graph.add(weights_graph);
    gtsam::Values values;
    gtsam::NonlinearFactorGraph graph;
    graph.add(graph_values.first);
    values.insert_or_assign(graph_values.second);

    std::cout << "Graph: " << graph.size() << std::endl;
    prx::fg::utilities::values_to_file<Eigen::VectorXd, basis_vector_t>(values,
                                                                        prx::out_path + "mj_friction_map_values.txt");
    graph.printErrors(values, "Errors", symbol_factory_t::formatter);
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
    // graph.printErrors(values, "Errors", symbol_factory_t::formatter);
    auto iter_results = fg::utilities::optimize_and_log(optimizer, lm_params, logs["fg_log"], fg_iterations);
    logs["thetas_error_log"].log(idx, optimizer.error());
    fg_iterations += optimizer.iterations();

    prx::symbol_factory_t::symbols_to_file();
    // results.print("Results", prx::symbol_factory_t::formatter);
    // gtsam::Marginals marginals{ graph, iter_results, gtsam::Marginals::Factorization::QR };
    // graph.saveGraph(fg_graph_file, results, prx::key_formatter, graph_formatter);

    // gtsam::KeyVector basis_joint;
    // for (auto basis_symbol : basis_used)
    // {
    //   basis_joint.push_back(basis_symbol.first);
    //   std::cout << prx::symbol_factory_t::formatter(basis_symbol.first) << "\t"
    //             << iter_results.at<friction_vector_t>(basis_symbol.first) << "\t"
    //             << marginals.marginalInformation(basis_symbol.first) << std::endl;
    //   auto basis_inf_mat =
    //   gtsam::noiseModel::Gaussian::Information(marginals.marginalInformation(basis_symbol.first));
    //   basis_covariances[basis_symbol.first] = basis_inf_mat;
    // }
    // marginals.jointMarginalInformation(basis_joint).print("marginal", prx::symbol_factory_t::formatter);
    // std::cout << "JointInf:\n" << marginals.jointMarginalInformation(basis_joint).fullMatrix() << std::endl;

    // logger.log(std::to_string(extra_iters + nl_opt.iterations()), newError);
    // const basis_vector_t new_frictions{ results.at<basis_vector_t>(param_symbol_basis) };
    // prx::friction_map::update_friction_grid(frictions_grid, new_frictions);
    // prx::fg::utilities::values_to_file<basis_vector_t>(results, prx::out_path + "friction_maps/mj_friction_map.txt");
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
    prx::fg::create_gml_file(graph, results, prx::out_path + "friction_maps/mj_friction_map.gml", variables_positions);
  }
  // const prx_symbol_t param_symbol_basis{ symbol_factory_t::create_hashed_symbol("param_basis", 0) };
  // auto basis = results.at<basis_vector_t>(param_symbol_basis);
  for (auto x : prx::linspace<double>(-XMAX, XMAX, 100))
  {
    for (auto y : prx::linspace<double>(-YMAX, YMAX, 100))
    {
      if (visited_grid(x, y)[0] > 0)
      {
        Position position_0(x, y);
        Position position_1(x, y + length_1);
        Position position_2(x + length_0, y);
        Position position_3(x + length_0, y + length_1);
        BasisPosition basis_positions;
        basis_positions.row(0) = frictions_grid.unmap<Eigen::Vector2d>(position_0[0], position_0[1]);
        basis_positions.row(1) = frictions_grid.unmap<Eigen::Vector2d>(position_1[0], position_1[1]);
        basis_positions.row(2) = frictions_grid.unmap<Eigen::Vector2d>(position_2[0], position_2[1]);
        basis_positions.row(3) = frictions_grid.unmap<Eigen::Vector2d>(position_3[0], position_3[1]);
        auto weight = fg::friction_local_fusion_factor_t<TH_DIM, State, BasisPosition>::compute_weight(
            position_0, length_0, basis_positions);

        auto basis_0 = results.at<friction_vector_t>(basis_grid(position_0[0], position_0[1]));
        auto basis_1 = results.at<friction_vector_t>(basis_grid(position_1[0], position_1[1]));
        auto basis_2 = results.at<friction_vector_t>(basis_grid(position_2[0], position_2[1]));
        auto basis_3 = results.at<friction_vector_t>(basis_grid(position_3[0], position_3[1]));
        Eigen::Vector4d basis(basis_0[0], basis_1[0], basis_2[0], basis_3[0]);

        auto friction_at_xy = weight.dot(basis);
        logs["idd_friction_map"](x, y, friction_at_xy);
      }
      else
      {
        logs["idd_friction_map"](x, y, 1.0);
      }
    }
  }
  // prx::friction_map::compute_friction_map<basis_vector_t>(frictions_grid, logs["idd_friction_map"],
  //                                                         prx::linspace<double>(-XMAX, XMAX, 50),
  //                                                         prx::linspace<double>(-YMAX, YMAX, 50));

  basis_grid.to_file(prx::out_path + "friction_maps/small_frictions_grid_mj.txt", [&](const prx::prx_symbol_t& s) {
    if (results.exists(s))
      return results.at<friction_vector_t>(s)[0];
    else
      return -1.0;
  });
  visited_grid.to_file(prx::out_path + "friction_maps/small_visited_grid_mj.txt",
                       [&](const friction_vector_t& v) { return v.transpose(); });
  for (auto& logger_pair : logs)
  {
    std::cout << logger_pair.first << " " << logger_pair.second.get_filename() << "\n";
    logger_pair.second.close();
  }
  return 0;
}
