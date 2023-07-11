#include "prx/mujoco/mj_simulator.hpp"
#include "prx/simulation/playback/plan.hpp"
#include <iostream>
#include <fstream>
#include <string>

using namespace prx;

std::string to_zero_lead(const int value, const unsigned precision)
{
  std::ostringstream oss;
  oss << std::setw(precision) << std::setfill('0') << value;
  return oss.str();
}

int main(int argc, char** argv)
{
  std::string params_file = "executables/factor_graphs/random_trajectory_collection.yaml";

  param_loader params(params_file, argc, argv);

  std::string model_filename = params["model_file"].as<std::string>();
  const bool visualize = params["visualize"].as<bool>();

  std::shared_ptr<mujoco_simulator_t> sim = std::make_shared<mujoco_simulator_t>(model_filename, visualize);
  sim->init_simulator();

  auto context = sim->get_context("mujoco");
  auto sg = context.first;
  auto ss = sg->get_state_space();
  auto cs = sg->get_control_space();

  init_random(params["random_seed"].as<int>());
  space_point_t start = ss->make_point();
  space_point_t zero_start = ss->make_point();

  // ss->copy(start, params["start_state"].as<std::vector<double>>());
  ss->copy_to(zero_start);
  std::string data_path{ params["data_path"].as<std::string>() };
  std::vector<double> x_bounds{ params["bounds/x"].as<std::vector<double>>() };
  std::vector<double> y_bounds{ params["bounds/y"].as<std::vector<double>>() };

  std::size_t total_trajectories{ params["trajectories_to_collect"].as<std::size_t>() };

  prx::precision = 20;
  prx::space_point_t zero_control = cs->make_point();
  cs->copy(zero_control, Eigen::VectorXd::Zero(cs->get_dimension()));
  const double plan_duration{ params["duration"].as<double>() };
  for (int i = 0; i < total_trajectories; ++i)
  {
    plan_t plan(cs);
    trajectory_t traj(ss);
    // double plan_duration{ prx::uniform_int_random(300, 300) / 100.0 };
    Eigen::Vector3d rand_ctrl{ Eigen::Vector3d::Random() };
    rand_ctrl /= rand_ctrl.norm();
    plan.copy_onto_back(rand_ctrl, plan_duration);

    ss->copy(start, zero_start);
    start->at(0) = prx::uniform_random(x_bounds[0], x_bounds[1]);
    start->at(1) = prx::uniform_random(y_bounds[0], y_bounds[1]);

    sg->propagate(start, zero_control, 0.1, start);
    sg->propagate(start, plan, traj);

    const std::string traj_path = data_path + "/traj_" + to_zero_lead(i, 5) + ".txt";
    const std::string plan_path = data_path + "/plan_" + to_zero_lead(i, 5) + ".txt";
    traj.to_file(traj_path);
    plan.to_file(plan_path);
  }
}
