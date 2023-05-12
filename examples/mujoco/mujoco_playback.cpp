#include "prx/mujoco/mj_simulator.hpp"
#include "prx/simulation/playback/plan.hpp"
#include <iostream>
#include <fstream>
#include <string>

using namespace prx;

int main(int argc, char** argv)
{
  std::string params_file = "examples/mujoco/playback.yaml";

  param_loader params(params_file, argc, argv);

  std::string control_filename = params["plan_file"].as<std::string>();
  std::string model_filename = params["model_file"].as<std::string>();

  std::shared_ptr<mujoco_simulator_t> sim = std::make_shared<mujoco_simulator_t>(model_filename);
  sim->init_simulator();

  auto context = sim->get_context("mujoco");
  auto ss = context.first->get_state_space();
  auto cs = context.first->get_control_space();

  init_random(params["random_seed"].as<int>());
  space_point_t start = ss->make_point();
  trajectory_t traj(ss);

  if (params["use_default_start"].as<bool>())
  {
    ss->copy_to(start);
    std::cout << "Start state: " << start << "\n";
  }
  else
  {
    ss->copy(start, params["start_state"].as<std::vector<double>>());
    ss->copy_from(start);
  }

  plan_t plan(cs);
  plan.from_file(control_filename);

  context.first->propagate(start, plan, traj);
  // usleep(int(1e6));
  // std::cout << ss->print_point(end, 3) << std::endl;

  traj.to_file(prx::out_path + "traj_playback.txt");
}