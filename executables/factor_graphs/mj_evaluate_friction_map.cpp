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
#include "prx/utilities/general/constants.hpp"
#include "prx/utilities/general/csv_reader.hpp"
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

#include "prx/mujoco/mj_simulator.hpp"
#include "prx/mujoco/plants/mj_friction_plant.hpp"
#include <gtsam/nonlinear/Marginals.h>

#include "friction_maps.hpp"

using namespace prx;

const int XMAX{ 20 };
const int YMAX{ 20 };
const int GRID_DIVISIONS{ 10 };
const int BASIS_DIM{ (GRID_DIVISIONS + 1) * (GRID_DIVISIONS + 1) };

const Eigen::Index TH_DIM{ Eigen::Dynamic };

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
  const std::string friction_map_file{ params["friction_map_file"].as<std::string>() };

  std::shared_ptr<mujoco_simulator_t> sim = std::make_shared<mujoco_simulator_t>(model_filename, visualize);

  std::shared_ptr<prx::mujoco_plant_t> mj_plant = std::make_shared<prx::mujoco_plant_t>("mujoco_plant");

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

  using State = Eigen::VectorXd;
  // using Control = typename PropagationMj::Control;
  // // using Time = typename PropagationMj::Time;
  // using Theta = typename PropagationMj::Theta;
  // using MjFunction = typename PropagationMj::MjFunction;

  logger_t logs;
  // friction_map::init_logmap(logs);

  prx::utilities::csv_reader_t reader(friction_map_file, ' ');
  prx_assert(reader.has_next_line(), "Empty file:" << friction_map_file);
  auto line = reader.next_line();
  basis_vector_t basis_from_file{};
  for (int i = 0; i < basis_from_file.size(); ++i)
  {
    basis_from_file[i] = prx::utilities::convert_to<double>(line[i + 1]);
  }

  const std::vector<std::pair<double, double>> env_bounds{ std::make_pair(-XMAX, XMAX), std::make_pair(-YMAX, YMAX) };
  prx::regular_grid_t<friction_vector_t, 2> frictions_grid{ env_bounds, GRID_DIVISIONS };

  friction_map::basis_vector_to_grid(basis_from_file, frictions_grid);

  int floor_id{ 0 };
  int total_geoms{ sim->_mj_model->ngeom };
  for (int i = 0; i < total_geoms; ++i)
  {
    std::string g1 = std::string(sim->_mj_model->names + sim->_mj_model->name_geomadr[i]);
    if (g1 == "floor0")
      floor_id = i;
  }
  // MjFunction fg_mjfn = [&](const State& x0, const State& x1, const Control& u, const Theta& th) {
  //   // sim->_mj_model->geom_friction[floor_id + 2] = th[0];
  //   ps->copy_from(th);
  // };

  std::size_t total_plan_trajectories_files{ params["total_plan_traj_files_to_use"].as<std::size_t>() };
  std::string data_path{ params["data_path"].as<std::string>() };

  plan_t plan(cs);
  trajectory_t trajectory_real(ss);
  trajectory_t trajectory_eval(ss);

  std::size_t fg_iterations{ 0 };
  std::vector<prx::prx_symbol_t> weights_used;
  State xt;
  for (std::size_t idx = 0; idx < total_plan_trajectories_files; ++idx)
  {
    const std::string traj_path = data_path + "/traj_" + to_zero_lead(idx, 5) + ".txt";
    const std::string plan_path = data_path + "/plan_" + to_zero_lead(idx, 5) + ".txt";
    plan.clear();
    trajectory_real.clear();
    plan.from_file(plan_path);
    trajectory_real.from_file(traj_path);
    plan.expand();
    PRX_DEBUG_VAR_2(plan.size(), trajectory_real.size());
    if (plan.size() + 1 != trajectory_real.size())
    {
      prx_warn("Plan/Traj " << idx << "mismatch on sizes");
      continue;
    }

    xt = trajectory_real[0]->vector();
    for (auto& step : plan)
    {
      // Compute th0 given the states
      // const basis_vector_t weights{ friction_map::weights_vector_given_state<basis_vector_t>(frictions_grid, xt) };
      // th0 = weights.dot(thetas);

      // Propagate given x1p <- (x0, u0, th0)
      // ps->copy_from(th);
      // sg->propagate(x0, plan, trajectory_eval);
      // x1 = trajectory_real.back();
      // x1p = trajectory_eval.back();

      // const double error{ (x1 - x1p).norm() };
    }
  }

  // const prx_symbol_t param_symbol_basis{ symbol_factory_t::create_hashed_symbol("param_basis", 0) };
  // auto basis = results.at<basis_vector_t>(param_symbol_basis);

  // prx::friction_map::compute_friction_map<basis_vector_t>(frictions_grid, logs["idd_friction_map"],
  //                                                         prx::linspace<double>(-XMAX, XMAX, 20),
  //                                                         prx::linspace<double>(-YMAX, YMAX, 20));

  // for (auto& logger_pair : logs)
  // {
  //   std::cout << logger_pair.first << " " << logger_pair.second.get_filename() << "\n";
  //   logger_pair.second.close();
  // }
  return 0;
}