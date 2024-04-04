// #ifndef TORCH_NOT_BUILT
#include "prx/utilities/defs.hpp"
#include "prx/mujoco/mj_simulator.hpp"
#include "prx/planning/planners/dirt.hpp"
#include "prx/mujoco/mj_utils.hpp"
#include "prx/utilities/heuristics/pose_utils.hpp"
#include "prx/utilities/heuristics/roadmap.hpp"

#include <boost/filesystem.hpp>
using namespace boost::filesystem;

using namespace prx;

int main(int argc, char* argv[])
{
  param_loader params;
  if (argc < 2)
  {
    params = param_loader("examples/JIST/heuristic_test.yaml");
  }
  else
  {
    params = param_loader(argv[1]);
  }
  init_random(params["random_seed"].as<int>());

  std::shared_ptr<prx::mujoco_simulator_t> sim =
      std::make_shared<prx::mujoco_simulator_t>(params["scene_xml_path"].as<std::string>());
  sim->init_simulator();

  auto context = sim->get_context("mujoco");
  auto ss = context.first->get_state_space();
  auto cs = context.first->get_control_space();


  std::shared_ptr<prx::mujoco_simulator_t> ee_sim =
      std::make_shared<prx::mujoco_simulator_t>(params["heuristic_xml_path"].as<std::string>());
  ee_sim->init_simulator();

  auto ee_context = sim->get_context("mujoco");
  auto ee_ss = context.first->get_state_space();
  auto ee_cs = context.first->get_control_space();

  for (int i = 0; i < 100; i++)
  {
    sim->step_simulation();
    ee_sim->step_simulation();
  }

  // mj_kinematics(sim->m, sim->d);
  std::vector<std::string> joint_names = params["joint_names"].as<std::vector<std::string>>();
    
  std::string hand = params["forward_name"].as<std::string>();
  std::vector<std::string> ee_names = params["end_effector"].as<std::vector<std::string>>();

  std::cout << "Arm simulation" << std::endl;
  for(int i = 0; i < sim->m->nbody; i++){
    int body_id = sim->m->body_jntadr[i];
    int qpos_adr = sim->m->jnt_qposadr[body_id];
    std::cout << mj_id2name(sim->m, mjOBJ_BODY, i) << ": " << i << ", " << qpos_adr << std::endl;
  }
  std::cout << "\n" << std::endl;

  std::cout << "End-effector simulation" << std::endl;
  for(int i = 0; i < ee_sim->m->nbody; i++){
    int body_id = ee_sim->m->body_jntadr[i];
    int qpos_adr = ee_sim->m->jnt_qposadr[body_id];
    std::cout << mj_id2name(ee_sim->m, mjOBJ_BODY, i) << ": " << i << ", " << qpos_adr << std::endl;
  }
  std::cout << "\n" << std::endl;

  std::cout << "End of program!" << std::endl;
}