// #ifndef TORCH_NOT_BUILT
#include "prx/utilities/defs.hpp"
#include "prx/mujoco/mj_simulator.hpp"
#include "prx/planning/planners/dirt.hpp"

#include "prx/mujoco/mj_utils.hpp"
#include "prx/mujoco/mj_manipulation.hpp"

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
  std::string hand = params["end_effector"].as<std::string>();
  // auto test = ;
  Eigen::Vector<double, 7> q_test(params["test_config"].as<std::vector<double>>().data());
  auto pose = forward_kinematics(sim->m, sim->d, qpos_inds, hand, q_test);
  
  for (auto val : pose){
    std::cout << val << "\t";
  }
  std::cout << std::endl;

  space_point_t point = ss->make_point();
  ss->sample(point);
  std::cout << point << std::endl;
  
  std::vector<std::string> ee_names = params["end_effector_bodies"].as<std::vector<std::string>>();

  std::cout << point->get_dim() << std::endl;
  distance_function_t distance_function = [&](const space_point_t& a, const space_point_t& b) {
    double dist = 0;
    for (auto end_effector_body : ee_names){
      auto pose_a = forward_kinematics(sim->m, sim->d, qpos_inds, end_effector_body, a);
      auto pose_b = forward_kinematics(sim->m, sim->d, qpos_inds, end_effector_body, b);

      double euclidean = 0;
      for (int i = 0; i < 3; i++){
        euclidean += std::pow(pose_a[i] - pose_b[i], 2.0);
      }
      euclidean = std::sqrt(euclidean);

      dist += euclidean;
    }

    return dist;
  };

  // roadmap_t heuristic{};

  // std::cout << heuristic.
  
  std::cout << "End of program!" << std::endl;
}