#include <iostream>

#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/type_conversions.hpp"
#include "prx/factor_graphs/lie_groups/se3.hpp"
#include "prx/factor_graphs/factors/se3_observation.hpp"
#include "prx/simulation/controllers/lqr.hpp"

#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/rrt_star.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/planners/planner.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/simulation/collision_checking/collision_group.hpp"

#include "prx/factor_graphs/factors/obstacle_factor.hpp"
#include "prx/factor_graphs/utilities/default_parameters.hpp"
#include "prx/factor_graphs/lie_groups/screw_axis.hpp"
#include "prx/factor_graphs/lie_groups/lie_integrator.hpp"
#include "prx/factor_graphs/factors/screw_smoothing.hpp"
#include "prx/factor_graphs/utilities/values_utilities.hpp"
#include "prx/factor_graphs/plants/SE2_rigid_body.hpp"

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/basis/FitBasis.h>
#include <gtsam/basis/Chebyshev2.h>

using SF = prx::fg::symbol_factory_t;
using Translation = Eigen::Vector3d;
using SE3ObsFactor = prx::fg::SE3_observation_factor_t;
using ScrewAxis = prx::fg::screw_axis_t;
using SE3 = prx::fg::se3_t;
using Integrator = prx::fg::lie_integration_factor_t<SE3, ScrewAxis>;
using ScrewSmothing = prx::fg::screw_smoothing_factor_t;
using Bars = std::tuple<SE3, SE3, SE3>;
using BarEndCaps = std::pair<Translation, Translation>;

int main(int argc, char* argv[])
{
  prx::param_loader params{};
  params["controller"].set("LQR");
  params["/checker/time"].set(5);
  params["order"].set(1);
  params["environment"].set("environments/empty.yaml");

  params.add_opts(argc, argv);

  const double time_limit{ params["/checker/time"].as<double>() };
  const int order{ params["order"].as<int>() };
  const int dimension{ order * 3 };

  prx_assert(order == 1 or order == 2, "'order' of plant is either 1 or 2.");
  const std::string plant_name{ order == 1 ? "SE2_rigid_body_1st_order" : "SE2_rigid_body_2nd_order" };
  prx::system_ptr_t plant{ prx::system_factory_t::create_system(plant_name, plant_name) };

  const prx::PairNameObstacles obstacles{ prx::load_obstacles(params["environment"].as<>()) };
  std::vector<std::shared_ptr<prx::movable_object_t>> obstacle_list{ obstacles.second };
  std::vector<std::string> obstacle_names{ obstacles.first };

  prx::world_model_t world_model({ plant }, { obstacle_list });
  world_model.create_context("context", { plant_name }, { obstacle_names });
  prx::simulation_context context{ world_model.get_context("context") };

  std::shared_ptr<prx::system_group_t> sg{ prx::system_group(context) };
  std::shared_ptr<prx::collision_group_t> cg{ prx::collision_group(context) };

  prx::space_t* ss{ sg->get_state_space() };

  prx::controller_ptr_t ctrl{ nullptr };
  if (params["controller"].as<>() == "LQR")
  {
    const Eigen::MatrixXd Q{ Eigen::MatrixXd::Identity(dimension, dimension) };
    const Eigen::MatrixXd R{ Eigen::MatrixXd::Identity(3, 3) };
    std::shared_ptr<prx::lqr_t> lqr{ std::make_shared<prx::lqr_t>(plant, Q, R, "LQR") };
    lqr->set_goal(Eigen::VectorXd::Zero(dimension));
    ctrl = lqr;
  }

  prx::condition_check_t checker("time", time_limit);

  prx::space_point_t start_state{ ss->make_point() };
  prx::trajectory_t traj(ss);

  sg->propagate(start_state, ctrl, checker, traj);

  prx::three_js_group_t* vis_group = new prx::three_js_group_t({ plant }, { obstacle_list });
  std::string body_name{ "body" };

  vis_group->add_detailed_vis_infos(prx::info_geometry_t::FULL_LINE, traj, body_name, ss);
  vis_group->add_animation(traj, ss, start_state);
  vis_group->output_html("se2_plant.html");

  delete vis_group;

  return 0;
}
