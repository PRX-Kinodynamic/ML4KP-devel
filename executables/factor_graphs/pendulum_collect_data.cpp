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

template <typename Plant>
void process_traj(prx::trajectory_t& traj, prx::plan_t& plan, Plant plant, const double dt, std::ofstream& ofs)
{
  prx::constants::precision = 10;
  State xd;
  const double length{ 0.5 };
  const double gravity{ 9.81 };
  const double total_duration{ plan.duration() };

  for (double ti = 0; ti < total_duration - dt; ti += dt)
  {
    const double dt01{ ti / total_duration };
    // PRX_DBG_VARS(ti, dt, dt01, traj.size());
    const prx::space_point_t xi{ traj.at(dt01, true) };
    const prx::space_point_t ui{ plan.at(ti) };
    const double theta1{ xi->at(0) };
    const double Gt{ gravity / length * std::sin(theta1) };

    plant->get_state_space()->copy_from(xi);
    plant->get_control_space()->copy_from(ui);
    plant->compute_derivative();
    plant->get_derivative_space()->copy_to(xd);

    const double accel{ xd[1] - Gt };  // xd[0] is x1[1]

    ofs << xi << " ";
    ofs << ui << " ";
    ofs << dt << " ";
    ofs << Gt << " ";
    ofs << accel << " ";
    ofs << "\n";
  }
  ofs << "\n";
}

int main(int argc, char* argv[])
{
  prx::param_loader params{};
  params["min_steps"].set(3);
  params["max_steps"].set(10);
  params["tmin"].set(0.2);
  params["tmax"].set(1.0);
  params["total_trajs"].set(100);
  params["dt"].set(0.1);
  params["out"].set(prx::out_path + "/pendulum_accel_data.txt");
  params.add_opts(argc, argv);

  prx::simulation_step = 0.01;
  const std::string plant_name{ "pendulum" };
  auto system = prx::system_factory_t::create_system(plant_name, plant_name);
  auto plant = std::dynamic_pointer_cast<prx::plant_t>(system);
  prx_assert(plant != nullptr, "Plant is nullptr!");

  prx::world_model_t world_model({ plant }, {});
  world_model.create_context("context", { plant_name }, {});
  auto context = world_model.get_context("context");
  std::shared_ptr<prx::system_group_t> sg{ prx::system_group(context) };

  prx::space_t* ss{ sg->get_state_space() };
  prx::space_t* cs{ sg->get_control_space() };
  prx::space_t* ps{ sg->get_parameter_space() };

  std::ofstream ofs(params["out"].as<>().c_str());

  prx::plan_t plan(cs);
  prx::trajectory_t traj(ss);
  prx::space_point_t x0{ ss->make_point() };

  const std::size_t min_steps{ params["min_steps"].as<std::size_t>() };
  const std::size_t max_steps{ params["max_steps"].as<std::size_t>() };
  const double tmin{ params["tmin"].as<double>() };
  const double tmax{ params["tmax"].as<double>() };
  const double dt{ params["dt"].as<double>() };

  const int total_trajs{ params["total_trajs"].as<int>() };

  const double inf{ std::numeric_limits<double>::infinity() };
  for (int i = 0; i < total_trajs; ++i)
  {
    ss->set_bounds({ -PRX_PI, -2 * PRX_PI }, { PRX_PI, 2 * PRX_PI });
    ss->sample(x0);
    ss->set_bounds({ -PRX_PI, -inf }, { PRX_PI, inf });
    plan.random(min_steps, max_steps, tmin, tmax);
    sg->propagate(x0, plan, traj);
    process_traj(traj, plan, plant, dt, ofs);
  }
  return 0;
};