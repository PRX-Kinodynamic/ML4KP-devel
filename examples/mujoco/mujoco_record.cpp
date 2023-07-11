#include "prx/mujoco/mj_simulator.hpp"
#include "prx/planning/planners/dirt.hpp"

#include <fstream>

using namespace prx;

int main(int argc, char* argv[])
{
  init_random(210896);
  const std::string params_file{ "examples/mujoco/mujoco_record.yaml" };
  param_loader params(params_file, argc, argv);
  const bool visualize = params["visualize"].as<bool>();

  const std::string mj_plant_filepath{ params["model_file"].as<>() };
  std::shared_ptr<mujoco_simulator_t> sim = std::make_shared<mujoco_simulator_t>(mj_plant_filepath, visualize);
  sim->init_simulator();

  auto context = sim->get_context("mujoco");
  auto sg = sim->get_context("mujoco");
  auto ss = context.first->get_state_space();
  auto cs = context.first->get_control_space();

  plan_t plan(cs);
  trajectory_t traj(ss);
  auto start = ss->make_point();

  plan.from_file(params["plan_filename"].as<>());
  PRX_DEBUG_VAR_1(plan);
  sim->set_video_name(params["video_name"].as<>());

  for (double i = 0; i < 1.0 / simulation_step; i += 1)
  {
    sim->step_simulation(propagate_step::FIRST_STEP);
  }
  ss->copy_to(start);
  PRX_DEBUG_VAR_1(start);
  sim->set_record_video(params["record_video"].as<bool>());
  context.first->propagate(start, plan, traj);

  sim->close_video();
}