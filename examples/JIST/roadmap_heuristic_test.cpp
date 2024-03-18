// #ifndef TORCH_NOT_BUILT
#include "prx/utilities/defs.hpp"
#include "prx/mujoco/mj_simulator.hpp"
#include "prx/planning/planners/dirt.hpp"
#include "prx/mujoco/mj_utils.hpp"

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
      std::make_shared<prx::mujoco_simulator_t>(params["xml_path"].as<std::string>());
  sim->init_simulator();

  auto context = sim->get_context("mujoco");
  auto ss = context.first->get_state_space();
  auto cs = context.first->get_control_space();


  for (int i = 0; i < 100; i++)
  {
    sim->step_simulation();
  }

  // mj_kinematics(sim->m, sim->d);
  std::vector<std::string> joint_names = params["joint_names"].as<std::vector<std::string>>();
  auto qpos_inds = get_qpos_indices(sim->m, mjOBJ_JOINT, joint_names);
  auto pose = forward_kinematics(sim->m, sim->d, qpos_inds, params["end_effector"].as<std::string>(), params["test_config"].as<std::vector<double>>());
  
  for (auto val : pose){
    std::cout << val << "\t";
  }
  std::cout << std::endl;
  // std::vector<std::string> joint_names = params["joint_names"].as<std::string>()
  
  std::cout << "End of program!" << std::endl;
}