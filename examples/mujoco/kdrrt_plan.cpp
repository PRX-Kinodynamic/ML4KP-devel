// #ifndef TORCH_NOT_BUILT
#include "prx/utilities/defs.hpp"
#include "prx/mujoco/mj_simulator.hpp"
#include "prx/planning/planners/rrt.hpp"

#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>

using namespace boost::filesystem;

using namespace prx;

int main(int argc, char* argv[])
{
  param_loader params;

  params = param_loader("examples/JIST/push.yaml");
  init_random(params["random_seed"].as<int>());

  bool visualize = params["visualize"].as<bool>();
  bool visualize_tree = params["visualize_tree"].as<bool>();

  std::vector<std::pair<std::string, std::string>> ignored_pairs =
      params["ignored_pairs"].as<std::vector<std::pair<std::string, std::string>>>();

  std::shared_ptr<prx::mujoco_simulator_t> sim =
      std::make_shared<prx::mujoco_simulator_t>(params["xml_path"].as<std::string>(), visualize);
  sim->init_simulator();

  sim->add_pair(ignored_pairs);

  auto context = sim->get_context("mujoco");
  auto ss = context.first->get_state_space();
  auto cs = context.first->get_control_space();

  for (int i = 0; i < 5; i++)
  {
    sim->step_simulation();
  }
  std::cout << ss->print_memory() << std::endl;

  rrt_t rrt(params["planner_name"].as<>());
  rrt_specification_t rrt_spec(context.first, context.second);

  float min_control_scaling = 0.1;
  float max_control_scaling = 1.0;
  rrt_spec.min_control_steps = min_control_scaling * (1.0 / simulation_step);
  rrt_spec.max_control_steps = max_control_scaling * (1.0 / simulation_step);

  rrt_spec.distance_function = [](const space_point_t& a, const space_point_t& b) {
    // std::cout << std::sqrt((b->at(0)-a->at(0))*(b->at(0)-a->at(0))+
    //                 (b->at(1)-a->at(1))*(b->at(1)-a->at(1))) << std::endl;

    return std::sqrt((b->at(0) - a->at(0)) * (b->at(0) - a->at(0)) + (b->at(1) - a->at(1)) * (b->at(1) - a->at(1)));
  };

  rrt_spec.blossom_number = params["blossom"].as<int>();
  //   rrt_spec.use_pruning = params["use_pruning"].as<bool>();
  //   double max_vel = params["random_seed"].as<double>();
  //   rrt_spec.h = [&](const space_point_t& s, const space_point_t& s2){
  //     return rrt_spec.distance_function(s, s2)/max_vel;
  //   };

  // rrt query
  rrt_query_t rrt_query(context.first->get_state_space(), context.first->get_control_space());

  rrt_query.get_visualization = params["visualize_tree"].as<bool>();

  rrt_query.goal_state = context.first->get_state_space()->make_point();

  rrt_query.start_state = context.first->get_state_space()->make_point();

  ss->copy_to(rrt_query.start_state);
  std::cout << "#### " << ss->print_point(rrt_query.start_state) << std::endl;

  // goal region radius
  rrt_query.goal_region_radius = params["goal_region_radius"].as<double>();

  // goal
  std::vector<double> goal_vec = params["goal_config"].as<std::vector<double>>();
  std::vector<double> env_xlim = params["env_xlim"].as<std::vector<double>>();
  std::vector<double> env_ylim = params["env_ylim"].as<std::vector<double>>();

  auto ub = ss->get_upper_bounds();
  auto lb = ss->get_lower_bounds();

  lb.at(0) = env_xlim[0];
  lb.at(1) = env_ylim[0];
  ub.at(0) = env_xlim[1];
  ub.at(1) = env_ylim[1];
  ss->set_bounds(lb, ub);

  for (int i = 0; i < ss->get_dimension(); i++)
  {
    std::cout << "dim " << i << " : " << lb.at(i) << " " << ub.at(i) << std::endl;
  }

  for (int i = 0; i < 2; i++)
  {
    // rrt_query.goal_state->at(i) = goal_vec[i];
    // rrt_query.goal_state->at(i) = rrt_query.start_state->at(i) + 0.1;
    rrt_query.goal_state->at(i) = goal_vec[i];
  }

  double goal_bias = params["goal_bias"].as<double>();
  rrt_spec.sample_state = [&](space_point_t& s) {
    if (uniform_random() < goal_bias)
    {
      ss->copy_point(s, rrt_query.goal_state);
    }
    else
    {
      default_sample_state(s, ss);
    }
  };

  // check whether sampled state is in goal region
  rrt_query.goal_check = [&](const space_point_t& point) {
    return rrt_spec.distance_function(point, rrt_query.goal_state) < rrt_query.goal_region_radius;
  };

  condition_check_t checker(params["checker_type"].as<>(), params["checker_value"].as<int>());  //

  rrt.link_and_setup_spec(&rrt_spec);

  rrt.preprocess();

  rrt.link_and_setup_query(&rrt_query);

  rrt.resolve_query(&checker);

  rrt.fulfill_query();

  unsigned count = 0;
  std::string output_folder = params["output_tree_folder"].as<std::string>();

  for (const auto& entry : directory_iterator(output_folder))
  {
    remove(entry.path());
  }

  for (auto traj : rrt_query.tree_visualization)
  {
    std::string filename = output_folder + std::to_string(count) + ".txt";
    std::ofstream fout(filename);
    fout << traj.print(16);
    count++;
  }

  prx::constants::precision = 16;
  if (params["output_plan"].as<bool>())
  {
    rrt_query.solution_plan.to_file(params["output_plan_file"].as<std::string>());
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