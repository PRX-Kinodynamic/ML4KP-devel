#include "prx/mujoco/mj_simulator.hpp"
#include "prx/simulation/playback/plan.hpp"
#include "prx/utilities/general/csv_reader.hpp"

#include <iostream>
#include <fstream>
#include <string>

using namespace prx;
double length;
extern mjtNum mjcb_act_dyn_user(const mjModel* model, const mjData* data, int id)
{
  const int nv{ model->nv };
  const int nc{ data->nefc };
  Eigen::Map<Eigen::VectorXd> c_vector(data->qfrc_bias, nv, 1);
  Eigen::Map<Eigen::VectorXd> tau_vector(data->qfrc_actuator, nv, 1);
  Eigen::Map<Eigen::VectorXd> vel_vector(data->qvel, nv, 1);
  Eigen::Map<Eigen::MatrixXd> jac_vector(data->efc_J, nc, nv);
  Eigen::Map<Eigen::VectorXd> f_vector(data->efc_force, nc, 1);
  Eigen::MatrixXd M_mat{ Eigen::MatrixXd::Zero(nv, nv) };
  mj_fullM(model, M_mat.data(), data->qM);
  // void mj_mulJacVec(const mjModel* m, mjData* d, mjtNum* res, const mjtNum* vec);

  auto LH = M_mat * vel_vector + c_vector;
  auto RH = jac_vector.transpose() * f_vector;

  PRX_DEBUG_VAR_1(LH.transpose());
  PRX_DEBUG_VAR_1(RH.transpose());
  PRX_DEBUG_VAR_1(tau_vector.transpose());
  // return (LH - RH).norm();
  // mj_tendon(model, data);
  const double error{ 20 - data->ten_length[0] };
  length += error;
  PRX_DEBUG_VAR_2(error, length);
  return error;
}

int main(int argc, char** argv)
{
  length = 0;
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

  // init_random(params["random_seed"].as<int>());
  space_point_t start = ss->make_point();
  trajectory_t traj(ss);

  mjcb_act_dyn = mjcb_act_dyn_user;

  ss->copy_to(start);

  sim->set_record_video(params["record_video"].as<bool>());
  for (double i = 0; i < 5; i += prx::simulation_step)
  {
    sim->step_simulation(propagate_step::MIDDLE_STEP);
  }
  sim->close_video();
  // traj.to_file(prx::out_path + "traj_playback.txt");
}