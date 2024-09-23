#include <iostream>

#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/utilities/general/type_conversions.hpp"

#include "prx/simulation/collision_checking/collision_group.hpp"
#include "prx/simulation/controllers/lqr_controller.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/simulation/plants/plants.hpp"

#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/planner.hpp"

#include "prx/visualization/three_js_group.hpp"

#include "prx/factor_graphs/plants/SE2_rigid_body.hpp"

int main(int argc, char* argv[])
{
  prx::param_loader params{};
  params["controller"].set("LQR");
  params["/checker/time"].set(5);
  params["order"].set(1);
  params["environment"].set("environments/bug_trap.yaml");
  params["start_state"].set(std::vector<double>{ 0, 0, 0 });

  params.add_opts(argc, argv);

  prx::simulation_step = 0.01;
  const double time_limit{ params["/checker/time"].as<double>() };
  const int order{ params["order"].as<int>() };
  const int dimension{ order * 3 };
  const std::vector<double> start_state{ params["start_state"].as<std::vector<double>>() };
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
    using LQR = prx::simulation::lqr_controller_t<>;
    const Eigen::MatrixXd Q{ Eigen::MatrixXd::Identity(dimension, dimension) };
    const Eigen::MatrixXd R{ Eigen::MatrixXd::Identity(3, 3) };
    const Eigen::VectorXd x_goal{ { -3.5, 0.0, prx::constants::pi / 2.0 } };
    const Eigen::VectorXd u_goal{ Eigen::VectorXd::Zero(3) };
    ctrl = std::make_shared<LQR>(plant, "LQR", Q, R, x_goal, u_goal);

    // ctrl = lqr;
  }

  prx::condition_check_t checker("sim_time", time_limit);

  prx::space_point_t x0{ ss->make_point() };
  prx::trajectory_t traj(ss);

  x0->at(0) = start_state[0];
  x0->at(1) = start_state[1];
  x0->at(2) = start_state[2];

  sg->propagate(x0, ctrl, checker, traj);

  prx::three_js_group_t* vis_group = new prx::three_js_group_t({ plant }, { obstacle_list });
  std::string body_name{ plant_name + "/body" };

  vis_group->add_detailed_vis_infos(prx::info_geometry_t::FULL_LINE, traj, body_name, ss);
  vis_group->add_animation(traj, ss, x0);
  vis_group->output_html("se2_plant.html");

  delete vis_group;

  return 0;
}
