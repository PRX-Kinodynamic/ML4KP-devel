// #ifndef TORCH_NOT_BUILT
#include "prx/utilities/defs.hpp"
#include "prx/mujoco/mj_simulator.hpp"
#include "prx/planning/planners/dirt.hpp"

#include <boost/filesystem.hpp>
using namespace boost::filesystem;

using namespace prx;

int main(int argc, char* argv[])
{
  param_loader params;
  if (argc < 2)
  {
    params = param_loader("examples/JIST/franka_dirt.yaml");
  }
  else
  {
    params = param_loader(argv[1]);
  }
  init_random(params["random_seed"].as<int>());

  std::vector<std::pair<std::string, std::string>> ignored_pairs = params["ignored_pairs"].as<std::vector<std::pair<std::string, std::string>>>();

  std::shared_ptr<prx::mujoco_simulator_t> sim =
      std::make_shared<prx::mujoco_simulator_t>(params["xml_path"].as<std::string>());
  sim->init_simulator();

  sim->add_pair(ignored_pairs);

  auto context = sim->get_context("mujoco");
  auto ss = context.first->get_state_space();
  auto cs = context.first->get_control_space();


  for (int i = 0; i < 100; i++)
  {
    sim->step_simulation();
  }

  dirt_t dirt(params["planner_name"].as<>());
  dirt_specification_t dirt_spec(context.first, context.second);

  dirt_spec.distance_function = [](const space_point_t& a, const space_point_t& b){
    return space_t::euclidean_2d(a, b, 0, 7);
  };

  std::vector<std::string> joint_names = params["joint_names"].as<std::vector<std::string>>();
  auto qpos_inds = get_qpos_indices(sim->m, mjOBJ_JOINT, joint_names);
  std::vector<std::string> ee_names = params["end_effector"].as<std::vector<std::string>>();

  dirt_spec.distance_function = [&](const space_point_t& a, const space_point_t& b) {
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
  // specify distance function


  float min_control_scaling = 0.5;
  float max_control_scaling = 1.0;
  dirt_spec.min_control_steps = min_control_scaling * (1.0 / simulation_step);
  dirt_spec.max_control_steps = max_control_scaling * (1.0 / simulation_step);

  // dirt query
  dirt_query_t dirt_query(context.first->get_state_space(), context.first->get_control_space());
  dirt_query.get_visualization = params["visualize"].as<bool>();

  dirt_query.goal_state = context.first->get_state_space()->make_point();
  dirt_query.start_state = context.first->get_state_space()->make_point();
  ss->copy_to(dirt_query.start_state);

  // goal region radius
  dirt_query.goal_region_radius = params["goal_region_radius"].as<double>();

  // goal
  std::vector<double> goal_vec = params["goal_config"].as<std::vector<double>>();
  for (int i = 0; i < goal_vec.size(); i++)
  {
      // rrt_query.goal_state->at(i) = goal_vec[i];
    // dirt_query.goal_state->at(i) = dirt_query.start_state->at(i) + 0.1;
    dirt_query.goal_state->at(i) = goal_vec[i];
  }

  // check whether sampled state is in goal region
  dirt_query.goal_check = [&](const space_point_t& point) {
    return dirt_spec.distance_function(point, dirt_query.goal_state) < dirt_query.goal_region_radius;
  };
  
  condition_check_t checker(params["checker_type"].as<>(), params["checker_value"].as<int>());  //'

  dirt.link_and_setup_spec(&dirt_spec);
  dirt.preprocess();
  dirt.link_and_setup_query(&dirt_query);
  dirt.resolve_query(&checker);
  dirt.fulfill_query();
  
  if(params["output_plan"].as<bool>()){
    dirt_query.solution_plan.to_file(params["output_path"].as<std::string>());
  }

  std::cout << "End of program!" << std::endl;
}

/*
#else
int main()
{
  std::cout << "Torch not built!" << std::endl;
}
#endif
*/