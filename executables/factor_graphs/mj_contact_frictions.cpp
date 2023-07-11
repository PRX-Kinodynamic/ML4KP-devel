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
  const bool visualize = params["visualize"].as<bool>();

  std::shared_ptr<mujoco_simulator_t> sim = std::make_shared<mujoco_simulator_t>(model_filename, visualize);
  sim->init_simulator();

  auto context = sim->get_context("mujoco");
  auto sg = context.first;
  auto ss = sg->get_state_space();
  auto cs = sg->get_control_space();

  init_random(params["random_seed"].as<int>());
  space_point_t start = ss->make_point();
  trajectory_t traj(ss);

  ss->copy(start, params["start_state"].as<std::vector<double>>());
  // ss->copy_to(start);

  plan_t plan(cs);
  plan.from_file(control_filename);
  plan.expand();
  // context.first->propagate(start, plan, traj);
  traj.copy_onto_back(start);
  std::size_t cont{ 0 };
  // int total_geoms{ sim->_mj_model->ngeom };
  // for (int i = 0; i < total_geoms; ++i)
  // {
  //   std::string g1 = std::string(sim->_mj_model->names + sim->_mj_model->name_geomadr[i]);
  //   if (g1 == "floor0")
  //     sim->_mj_model->geom_friction[i + 2] = 0.1;
  // }
  prx::precision = 10;
  sg->propagate(start, plan, traj);
  // for (const plan_step_t& step : plan)
  // {
  //   const double duration{ step.duration };
  //   prx_assert(duration >= 0.0, "Negative duration!");
  //   sg->propagate(start, step.control, duration, start);
  //   traj.copy_onto_back(start);
  //   int total_contacts{ sim->_mj_data->ncon };
  //   for (int i = 0; i < total_geoms; ++i)
  //   {
  //     std::string g1 = std::string(sim->_mj_model->names + sim->_mj_model->name_geomadr[i]);
  //   }
  //   for (int i = 0; i < total_contacts; ++i)
  //   {
  //     int id_geom1{ sim->_mj_data->contact[i].geom1 };
  //     int id_geom2{ sim->_mj_data->contact[i].geom2 };
  //   }
  // }
  // PRX_DEBUG_VAR_1(start);
  usleep(int(1e6));
  // std::cout << ss->print_point(end, 3) << std::endl;

  traj.to_file(params["out_traj"].as<>());
}