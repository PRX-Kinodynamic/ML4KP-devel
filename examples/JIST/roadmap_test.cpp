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


  for (int i = 0; i < 100; i++)
  {
    sim->step_simulation();
  }

  // mj_kinematics(sim->m, sim->d);
  std::vector<std::string> joint_names = params["joint_names"].as<std::vector<std::string>>();
  auto qpos_inds = get_qpos_indices(sim->m, mjOBJ_JOINT, joint_names);
  std::string hand = params["forward_name"].as<std::string>();

  std::vector<std::string> ee_names = params["end_effector"].as<std::vector<std::string>>();

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

  roadmap_t roadmap(params["planner_name"].as<>());
  roadmap_specification_t roadmap_spec(context.first, context.second);
  // std::cout << roadmap_spec.state_space << std::endl;
  roadmap_spec.distance_function = distance_function;

  roadmap_spec.state_to_config = [&](const space_point_t& state, space_point_t& config){
    // note: this part assumes that the joint angles in the state are at the beginning
    std::vector<double> state_vec(config->get_dim()); 
    for (int i = 0; i < state_vec.size(); i++){
      state_vec[i] = state->at(i);
    }
    auto pose = forward_kinematics(sim->m, sim->d, qpos_inds, hand, state_vec);
    prx_assert(config->get_dim() == pose.size(), "Incorrect dimensions for config");
    for (int i = 0; i < pose.size(); i++){
      config->at(i) = pose[i];
    }
    return config;
  };

  roadmap_spec.config_space = pose_space();

  // std::cout << "config space dims: " << roadmap_spec.config_space->get_dimension() << std::endl;

  roadmap_query_t roadmap_query(context.first->get_state_space(), context.first->get_control_space());
  roadmap_query.get_visualization = params["visualize"].as<bool>();

  roadmap_query.goal_state = context.first->get_state_space()->make_point();
  roadmap_query.start_state = context.first->get_state_space()->make_point();

  std::vector<double> goal_vec = params["goal_config"].as<std::vector<double>>();
  for (int i = 0; i < goal_vec.size(); i++)
  {
    // rrt_query.goal_state->at(i) = goal_vec[i];
    roadmap_query.goal_state->at(i) = goal_vec[i]; // rrt_query.start_state->at(i) + 0.1;
  }

  condition_check_t checker(params["checker_type"].as<>(), params["checker_value"].as<int>());  //'

  std::cout << "starting query" << std::endl;

  roadmap.link_and_setup_spec(&roadmap_spec);
  std::cout << "link_and_setup_spec" << std::endl;

  roadmap.preprocess();
  std::cout << "preprocess" << std::endl;

  roadmap.link_and_setup_query(&roadmap_query);
  std::cout << "link_and_setup_query" << std::endl;

  roadmap.resolve_query(&checker);
  std::cout << "resolve_query" << std::endl;

  roadmap.fulfill_query();
  std::cout << "fulfill_query" << std::endl;
  
  // roadmap_t heuristic{};

  // std::cout << heuristic.
  
  std::cout << "End of program!" << std::endl;
}