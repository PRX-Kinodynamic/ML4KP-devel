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
#include "prx/factor_graphs/utilities/utilities_functions.hpp"
#include "prx/factor_graphs/utilities/formatter.hpp"
#include "prx/factor_graphs/utilities/friction_map.hpp"

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

  std::unordered_map<std::string, logger_t> logs{};
  friction_map::init_logmap(logs, "batch_");

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
    th_transform[0] = std::exp(-th[0] * 0.1);
    // th_transform[0] = std::max(1.0 - 0.01 * th[0], 0.0);
    // th_transform[0] = th[0];
    // th_transform[0] = th[0] / 100;

    ps->copy_from(th_transform);
  };
  gtsam::Values results;
  gtsam::Values init_vals;
  gtsam::LevenbergMarquardtParams lm_params{ fg::utilities::default_levenberg_marquardt_parameters() };
  lm_params.setUseFixedLambdaFactor(true);
  lm_params.setMaxIterations(10000);
  lm_params.setRelativeErrorTol(1e-10);
  lm_params.setAbsoluteErrorTol(1e-10);
  lm_params.setlambdaUpperBound(1e64);
  lm_params.setVerbosityLM("LAMBDA");

  std::unordered_map<std::string, gtsam::noiseModel::Base::shared_ptr> noise_models;
  State noise{ State::Ones(ss_dim) * 1e-5 };
  noise.head(3) = Eigen::Vector3d::Ones() * 1e0;
  noise_models["state_space"] = gtsam::noiseModel::Diagonal::Sigmas(noise);
  noise_models["control_space"] = gtsam::noiseModel::Isotropic::Sigma(cs_dim, 1e-3);
  noise_models["positive_friction"] = gtsam::noiseModel::Isotropic::Sigma(1, 1e-5);
  noise_models["parameter_space"] = gtsam::noiseModel::Isotropic::Sigma(1, 1e0);
  noise_models["basis"] = gtsam::noiseModel::Isotropic::Sigma(BASIS_DIM, 0);
  noise_models["propagation"] = gtsam::noiseModel::Isotropic::Sigma(ss_dim, 1e-0);
  noise_models["friction_fusion"] = gtsam::noiseModel::Isotropic::Sigma(1, 1e0);
  // noise_models["weight"] = gtsam::noiseModel::Isotropic::Sigma(BASIS_DIM, 1e0);
  noise_models["positive_basis"] = gtsam::noiseModel::Isotropic::Sigma(BASIS_DIM, 1e-5);
  noise_models["guard"] = gtsam::noiseModel::Isotropic::Sigma(BASIS_DIM, 1e-5);
  noise_models["small_basis"] = gtsam::noiseModel::Isotropic::Sigma(1, 1e-5);
  // gtsam::SharedGaussian basis_nm = gtsam::noiseModel::Isotropic::Sigma(BASIS_DIM, 1e1);

  std::size_t total_plan_trajectories_files{ params["total_plan_traj_files_to_use"].as<std::size_t>() };
  std::size_t init_traj_id{ params["init_traj_id"].as<std::size_t>() };
  std::string data_path{ params["data_path"].as<std::string>() };

  plan_t plan(cs);
  trajectory_t trajectory(ss);

  const double length_0{ frictions_grid.get_cell_length(0) };
  const double length_1{ frictions_grid.get_cell_length(1) };

  prx::fg::mj_friction_map_t<TH_DIM, BASIS_DIM> mj_friction_map(sg, env_bounds, GRID_DIVISIONS, initial_friction_vec);
  mj_friction_map._fg_mjfn = fg_mjfn;
  std::size_t fg_iterations{ 0 };

  gtsam::Values batch_values;
  gtsam::NonlinearFactorGraph batch_graph;

  gtsam::Values batch_results;
  gtsam::Values previous_values;

  fg::formatter_t graph_formatter;

  // for (std::size_t batch_idx = 0; batch_idx < total_batches; ++batch_idx)
  // {
  for (std::size_t idx = init_traj_id; idx < init_traj_id + total_plan_trajectories_files; ++idx)
  {
    const std::string traj_path = data_path + "/traj_" + to_zero_lead(idx, 5) + ".txt";
    const std::string plan_path = data_path + "/plan_" + to_zero_lead(idx, 5) + ".txt";

    plan.clear();
    trajectory.clear();

    plan.from_file(plan_path);
    trajectory.from_file(traj_path);

    plan.expand();

    if (plan.size() + 1 != trajectory.size())
    {
      prx_warn("Plan/Traj " << idx << "mismatch on sizes");
      continue;
    }

    auto values = mj_friction_map.create_factor_graph(trajectory, plan, idx, batch_graph, previous_values);
    // batch_graph.saveGraph(prx::out_path + "friction_maps/batch_graph_" + std::to_string(idx) + ".txt", values,
    //                       prx::symbol_factory_t::formatter, graph_formatter);
    // batch_graph.add_factors(graph_values.first);
    batch_values.insert_or_assign(values);
    prx::symbol_factory_t::symbols_to_file();
  }
  // }
  std::cout << "Graph: " << batch_graph.size() << std::endl;
  batch_graph.saveGraph(prx::out_path + "friction_maps/batch_graph.txt", batch_values, prx::symbol_factory_t::formatter,
                        graph_formatter);
  prx::fg::utilities::values_to_file<Eigen::VectorXd, basis_vector_t>(batch_values, prx::out_path +
                                                                                        "batch_mj_friction_map_values."
                                                                                        "txt");
  gtsam::LevenbergMarquardtOptimizer optimizer(batch_graph, batch_values, lm_params);
  batch_results = fg::utilities::optimize_and_log(optimizer, lm_params, logs["fg_log"], fg_iterations);
  logs["thetas_error_log"].log(0, optimizer.error());

  std::function<std::tuple<bool, Position>(const gtsam::Values&, const gtsam::Key&)> variables_positions =
      [&](const gtsam::Values& values, const gtsam::Key& key)  // no-lint
  {
    bool bool_res{ false };
    Position pos_res{ Position::Zero() };
    if (mj_friction_map._symbol_positions.count(key) > 0)
    {
      bool_res = true;
      pos_res = 1000 * mj_friction_map._symbol_positions[key];
    }

    return std::make_tuple(bool_res, pos_res);
  };
  prx::fg::create_gml_file(batch_graph, batch_results, prx::out_path + "friction_maps/fg_batch.gml",
                           variables_positions);

  prx::fg::utilities::values_to_file<basis_vector_t>(results, prx::out_path +
                                                                  "friction_maps/"
                                                                  "batch_mj_friction_map.txt");

    return std::make_tuple(bool_res, pos_res);
  };
  prx::fg::create_gml_file(accum_fg, resulting_values, prx::out_path + "friction_maps/mj_friction_map.gml",
                           variables_positions);
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
        auto weight = prx::fg::compute_weight<Weights>(position_0, length_0, basis_positions);

        auto basis_0 = resulting_values.at<friction_vector_t>(basis_grid(position_0[0], position_0[1]));
        auto basis_1 = resulting_values.at<friction_vector_t>(basis_grid(position_1[0], position_1[1]));
        auto basis_2 = resulting_values.at<friction_vector_t>(basis_grid(position_2[0], position_2[1]));
        auto basis_3 = resulting_values.at<friction_vector_t>(basis_grid(position_3[0], position_3[1]));
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

  prx::friction_map::compute_friction_map<basis_vector_t>(frictions_grid, visited_grid, logs["idd_friction_map"],
                                                          prx::linspace<double>(-XMAX, XMAX, 20),
                                                          prx::linspace<double>(-YMAX, YMAX, 20));

  mj_friction_map.to_files(batch_results, "batch");

  // mj_friction_map._frictions_grid.to_file(prx::out_path + "friction_maps/batch_frictions_grid_mj.txt",
  //                                         [&](const friction_vector_t& v) { return v.transpose(); });
  // mj_friction_map._visited_grid.to_file(prx::out_path + "friction_maps/batch_visited_grid_mj.txt",
  //                                       [&](const friction_vector_t& v) { return v.transpose(); });
  for (auto& logger_pair : logs)
  {
    std::cout << logger_pair.first << " " << logger_pair.second.get_filename() << "\n";
    logger_pair.second.close();
  }
  return 0;
}
