// #ifndef TORCH_NOT_BUILT
#include "prx/utilities/defs.hpp"
#include "prx/mujoco/mj_simulator.hpp"
#include "prx/planning/planners/rrt.hpp"

#include <boost/filesystem.hpp>
using namespace boost::filesystem;

using namespace prx;

int main(int argc, char* argv[])
{
  param_loader params;
  if (argc < 2)
  {
    params = param_loader("examples/JIST/franka_rrt.yaml");
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

  rrt_t rrt(params["planner_name"].as<>());
  rrt_specification_t rrt_spec(context.first, context.second);

  // specify distance function
  rrt_spec.distance_function = [](const space_point_t& a, const space_point_t& b) {
    return space_t::euclidean_2d(a, b, 0, 7);
  };

  float min_control_scaling = 0.5;
  float max_control_scaling = 1.0;
  rrt_spec.min_control_steps = min_control_scaling * (1.0 / simulation_step);
  rrt_spec.max_control_steps = max_control_scaling * (1.0 / simulation_step);

  // rrt query ?
  rrt_query_t rrt_query(context.first->get_state_space(), context.first->get_control_space());
  rrt_query.get_visualization = true; //params["visualize"].as<bool>();

  rrt_query.goal_state = context.first->get_state_space()->make_point();
  rrt_query.start_state = context.first->get_state_space()->make_point();
  ss->copy_to(rrt_query.start_state);

  // goal region radius
  rrt_query.goal_region_radius = params["goal_region_radius"].as<double>();

  // goal
  std::vector<double> goal_vec = params["goal_config"].as<std::vector<double>>();
  for (int i = 0; i < goal_vec.size(); i++)
  {
    // rrt_query.goal_state->at(i) = goal_vec[i];
    rrt_query.goal_state->at(i) = goal_vec[i]; // rrt_query.start_state->at(i) + 0.1;
  }

  // check whether sampled state is in goal region
  rrt_query.goal_check = [&](const space_point_t& point) {
    return rrt_spec.distance_function(point, rrt_query.goal_state) < rrt_query.goal_region_radius;
  };
  
  condition_check_t checker(params["checker_type"].as<>(), params["checker_value"].as<int>());  //'

  std::cout << "starting!" << std::endl;
  rrt.link_and_setup_spec(&rrt_spec);
  std::cout << "link_and_setup_query finished" << std::endl;
  rrt.preprocess();
  rrt.link_and_setup_query(&rrt_query);
  rrt.resolve_query(&checker);
  rrt.fulfill_query();
  
  std::cout << "asdfasdf" << rrt_query.solution_plan.print(16) << std::endl;

  rrt_query.solution_plan.to_file(params["output_path"].as<std::string>());

  trajectory_t sol_traj = rrt_query.solution_traj;
  
  std::cout << sol_traj.size() << std::endl;

  // std::cout << sol_traj.print() << std::endl;
  // sim->set_goal(rrt_query.goal_state);

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