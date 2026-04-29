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
  params["duration"].set(5);
  params["environment"].set("environments/empty.yaml");
  params["start_state"].set(std::vector<double>{ 0, 0 });
  params["visualize"].set(true);
  params["total"].set(100);
  params.add_opts(argc, argv);

  prx::simulation_step = 0.01;
  const double time_limit{ params["duration"].as<double>() };
  const int dimension{ 2 };
  const std::vector<double> start_state{ params["start_state"].as<std::vector<double>>() };
  const std::string plant_name{ "pendulum" };
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
  prx::space_t* cs{ sg->get_control_space() };

  prx::controller_ptr_t ctrl{ nullptr };
  if (params["controller"].as<>() == "LQR")
  {
    using LQR = prx::simulation::lqr_controller_t<>;
    const Eigen::MatrixXd Q{ Eigen::MatrixXd::Identity(dimension, dimension) };
    const Eigen::MatrixXd R{ Eigen::MatrixXd::Identity(1, 1) };
    const Eigen::VectorXd x_goal{ Eigen::Vector2d::Zero() };
    const Eigen::VectorXd u_goal{ Eigen::VectorXd::Zero(1) };
    ctrl = std::make_shared<LQR>(plant, "LQR", Q, R, x_goal, u_goal);

    // ctrl = lqr;
  }

  prx::condition_check_t checker("sim_time", time_limit);

  prx::space_point_t x0{ ss->make_point() };
  prx::space_point_t ut{ cs->make_point() };
  prx::trajectory_t traj(ss);

  const std::string filename{ prx::out_path + "/pendulum_trajs.txt" };
  std::ofstream ofs_trajs(filename.c_str());
  ofs_trajs << "# x0 xdot0 u01 x1 xdot1\n ";

  const int total_trajs{ params["total"].as<int>() };
  for (int i = 0; i < total_trajs; ++i)
  {
    // x0->at(0) = start_state[0];
    // x0->at(1) = start_state[1];
    ss->sample(x0);
    traj.clear();
    // sg->propagate(x0, ctrl, checker, traj);
    ss->copy_from(x0);

    checker.reset();

    do
    {
      ofs_trajs << x0 << " ";
      ctrl->compute_controls();
      cs->copy_to(ut);
      sg->propagate_once();
      ss->copy_to(x0);
      ofs_trajs << ut << " " << x0 << "\n";
    } while (!checker.check());
    ofs_trajs << "\n";
  }

  // prx::three_js_group_t* vis_group = new prx::three_js_group_t({ plant }, { obstacle_list });
  // std::string body_name{ plant_name + "/ball" };

  // if (params["visualize"].as<bool>())
  // {
  //   vis_group->add_detailed_vis_infos(prx::info_geometry_t::FULL_LINE, traj, body_name, ss);
  //   vis_group->add_animation(traj, ss, x0);
  //   vis_group->output_html("pendulum_lqr.html");
  // }

  // delete vis_group;

  return 0;
}
