#include "prx/mujoco/mj_simulator.hpp"
#include "prx/simulation/playback/plan.hpp"
#include "prx/utilities/general/csv_reader.hpp"

#include <iostream>
#include <fstream>
#include <string>

using namespace prx;
using csv_reader_t = prx::utilities::csv_reader_t;

int main(int argc, char** argv)
{
  std::string params_file = "examples/mujoco/tensegrity_control.yaml";

  param_loader params(params_file, argc, argv);

  const std::string model_filename = params["model_file"].as<std::string>();
  const std::string controls_filename = params["controls_file"].as<std::string>();
  const bool visualize = params["visualize"].as<bool>();

  std::shared_ptr<mujoco_simulator_t> sim = std::make_shared<mujoco_simulator_t>(model_filename, visualize);
  sim->init_simulator();

  // y = 0.0006242x + 0.8466122

  auto context = sim->get_context("mujoco");
  auto ss = context.first->get_state_space();
  auto cs = context.first->get_control_space();

  csv_reader_t reader(controls_filename, ' ');
  plan_t plan(cs);
  double prev_time{ -1.0 };
  double ti{ 1.0 };
  space_point_t ctrl0 = cs->make_point();
  space_point_t ctrl1 = cs->make_point();
  space_point_t ctrl_ti = cs->make_point();
  Eigen::Vector<double, 12> cv(39, 39, 39, 39, 32, 32, 32, 32, 32, 32, 32, 32);

  // i<-real cable id
  // real_sim_tendon_mapping[i] -> sim id
  std::vector<int> real_sim_tendon_mapping = { 5, 6, 8, 7, 3, 11, 1, 9, 10, 4, 2, 12 };
  std::vector<int> sim_real_tendon_mapping = { 7, 11, 5, 10, 1, 2, 4, 3, 8, 9, 6, 12 };
  std::vector<int> stable_lengths = { 0, 32, 32, 32, 32, 37, 32, 37, 32, 32, 37, 37, 32 };

  using Ctrl = Eigen::Vector<double, 12>;
  Ctrl ctrl_v(37, 37, 37, 37, 32, 32, 32, 32, 32, 32, 32, 32);
  Ctrl ctrl_d(Ctrl::Zero());
  plan.copy_onto_back(ctrl_v, ti);
  cs->copy(ctrl0, ctrl_v);
  while (reader.has_next_line())
  {
    auto line = reader.next_line<double>();
    if (line.size() == 0)
      continue;
    if (prev_time > 0)
    {
      ti = line.back() - prev_time;
      cs->copy(ctrl1, ctrl_v);
      for (double inter = 0; inter < 1; inter += prx::simulation_step / ti)
      {
        cs->interpolate(ctrl0, ctrl1, inter, ctrl_ti);
        plan.copy_onto_back(ctrl_ti, prx::simulation_step);
      }
      cs->copy(ctrl0, ctrl1);
    }
    // cs->copy(ctrl_v, cv + 2 * Eigen::Vector<double, 12>::Random());
    for (std::size_t i = 0; i < line.size() - 1; ++i)
    {
      const int mapped_sim_real{ sim_real_tendon_mapping[i] };
      const int mapped_real_sim{ real_sim_tendon_mapping[i] };
      // ctrl_v[i] = stable_lengths[mapped_val];
      ctrl_v[i] = stable_lengths[mapped_sim_real] - (0.0006242 * line[mapped_real_sim - 1] + 0.8466122);
    }
    // PRX_DEBUG_VAR_1(ctrl_d.transpose());
    // PRX_DEBUG_VAR_1(ctrl_v.transpose());
    // ctrl_v -= ctrl_d;
    prev_time = line.back();
  }

  PRX_DEBUG_VAR_1(plan.size());
  PRX_DEBUG_VAR_1(plan);
  // init_random(params["random_seed"].as<int>());
  space_point_t start = ss->make_point();
  trajectory_t traj(ss);

  ss->copy_to(start);
  // std::cout << "Start state: " << start << "\n";

  sim->set_record_video(params["record_video"].as<bool>());
  // for (double i = 0; i < 5; i += prx::simulation_step)
  // {
  //   sim->step_simulation(propagate_step::MIDDLE_STEP);
  // }

  context.first->propagate(start, plan, traj);
  sim->close_video();
  // traj.to_file(prx::out_path + "traj_playback.txt");
}