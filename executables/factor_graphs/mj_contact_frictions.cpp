#include "prx/mujoco/mj_simulator.hpp"
#include "prx/simulation/playback/plan.hpp"
#include <iostream>
#include <fstream>
#include <string>

using namespace prx;

void insert_start_state(std::vector<double>& start_state, param_loader& params, std::vector<std::string> names)
{
  for (auto n : names)
  {
    /* code */
    auto partial_start_state = params["plant/start_state/" + n].as<std::vector<double>>();
    start_state.insert(start_state.end(), partial_start_state.begin(), partial_start_state.end());
  }
}

int main(int argc, char** argv)
{
  const std::string params_file{ "examples/mujoco/contact_frictions.yaml" };

  param_loader params(params_file, argc, argv);

  std::string control_filename = params["plan_file"].as<std::string>();
  std::string model_filename = params["plant/model_file"].as<std::string>();
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

  std::vector<double> start_state{};
  ss->copy_to(start);
  insert_start_state(start_state, params, { "xyz", "quat", "vels", "other" });
  ss->copy(start, start_state);
  PRX_DEBUG_VAR_1(start);
  // ss->copy_to(start);

  plan_t plan(cs);
  plan.from_file(control_filename);
  plan.expand();
  traj.copy_onto_back(start);
  std::size_t cont{ 0 };

  prx::precision = 10;
  sg->propagate(start, Eigen::VectorXd::Zero(cs->size()), 1, start);
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
  const double duration{ plan.duration() };
  const double frequency{ params["observation/frequency"].as<double>() };
  const std::string observ_file{ params["observation/filename"].as<std::string>() };

  std::ofstream ofs_obs;
  ofs_obs.open(observ_file.c_str(), std::ofstream::trunc);

  std::size_t idx{ 0 };
  for (double t = 0; t < duration; t += prx::simulation_step)
  {
    if (std::fmod(t, frequency) <= prx::simulation_step)
    {
      ofs_obs << traj[idx]->vector().head(3).transpose() << "\n";
    }
    idx++;
  }

  return 0;
}
