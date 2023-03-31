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
  auto sg = context.first;
  auto ss = sg->get_state_space();
  auto cs = sg->get_control_space();

  init_random(params["random_seed"].as<int>());
  space_point_t start = ss->make_point();
  trajectory_t traj(ss);

  ss->copy(start, params["start_state"].as<std::vector<double>>());
  ss->copy_from(start);

  plan_t plan(cs);
  plan.from_file(control_filename);
  plan.expand();
  // context.first->propagate(start, plan, traj);

  for (const plan_step_t& step : plan)
  {
    const double duration{ step.duration };
    prx_assert(duration >= 0.0, "Negative duration!");
    sg->propagate(start, step.control, duration, start);
    int total_contacts{ sim->_mj_data->ncon };
    int total_geoms{ sim->_mj_model->ngeom };
    std::cout << "Contacts: " << total_contacts << " total geoms: " << total_geoms << "\n";
    for (int i = 0; i < total_contacts; ++i)
    {
      int id_geom1{ sim->_mj_data->contact[i].geom1 };
      int id_geom2{ sim->_mj_data->contact[i].geom2 };
      std::string g1 = std::string(sim->_mj_model->names + sim->_mj_model->name_geomadr[id_geom1]);
      std::string g2 = std::string(sim->_mj_model->names + sim->_mj_model->name_geomadr[id_geom2]);
      std::cout << "geomsids: " << id_geom1 << " " << id_geom2 << ".\n";
      std::cout << "geoms: " << g1 << " " << g2 << ".\n";
      // std::cout << "Frictions: ";
      for (int j = 0; j < 5; ++j)
      {
        std::cout << sim->_mj_data->contact[i].friction[j] << " ";
      }
      std::cout << std::endl;
    }
  }
  PRX_DEBUG_VAR_1(start);
  usleep(int(1e6));
  // std::cout << ss->print_point(end, 3) << std::endl;

  traj.to_file(prx::out_path + "traj_playback.txt");
}