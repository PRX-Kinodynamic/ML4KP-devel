#include "prx/utilities/defs.hpp"
#include "prx/mujoco/mj_simulator.hpp"
// #include "prx/planning/planners/dirt.hpp"
#include "prx/planning/planners/prm.hpp"

using namespace prx;

int main(int argc, char* argv[])
{
  // load parameters from yaml
  param_loader params;
  params = param_loader("examples/tasks/prm.yaml");
  init_random(params["random_seed"].as<int>());

  std::string xml_path = params["xml_path"].as<std::string>();
  bool visualize_tree = params["visualize_tree"].as<bool>();
  bool visualize = params["visualize"].as<bool>();
  double goal_region_radius = params["goal_region_radius"].as<double>();
  std::vector<double> goal_vec = params["goal_config"].as<std::vector<double>>();
  std::vector<double> env_xlim = params["env_xlim"].as<std::vector<double>>();
  std::vector<double> env_ylim = params["env_ylim"].as<std::vector<double>>();

  condition_check_t checker(params["checker_type"].as<>(), params["checker_value"].as<int>());
  std::shared_ptr<prx::mujoco_simulator_t> sim = std::make_shared<prx::mujoco_simulator_t>(xml_path, visualize);
  sim->init_simulator();

  sim->add_pair({ { "ball", "case" } });
  sim->add_pair({
      { "ball", "wall_1" },
      { "ball", "wall_2" },
      { "ball", "wall_3" },
      { "ball", "wall_4" },
      { "ball", "wall_5" },
      { "ball", "wall_6" },
      // { "ball", "movable_cube1" },
      // { "ball", "movable_cube2" },
      // { "ball", "movable_cube3" },
  });

  for (int i = 0; i < 5; i++)
  {
    sim->step_simulation();
  }

  auto context = sim->get_context("mujoco");
  auto sg = context.first;
  auto ss = context.first->get_state_space();
  auto cs = context.first->get_control_space();

  // set bounds
  std::vector<double> ub = ss->get_upper_bounds();
  std::vector<double> lb = ss->get_lower_bounds();

  lb.at(0) = env_xlim[0];
  lb.at(1) = env_ylim[0];
  ub.at(0) = env_xlim[1];
  ub.at(1) = env_ylim[1];

  ss->set_bounds(lb, ub);

  // prm planner
  prm_t prm_planner("prm");

  prm_specification_t prm_spec(context.first, context.second);
  prm_spec.local_planner = [&](space_point_t& start, space_point_t& end, trajectory_t& traj, unsigned size) {
    traj.clear();
    // Given two states, start and end, and a number of steps size,
    // this function will interpolate a trajectory between the two states.
    for (unsigned i = 0; i < size; ++i)
    
    {
      traj.copy_onto_back(start);
      traj.at(i)->at(0) = start->at(0) + (end->at(0) - start->at(0)) * i / size;
      traj.at(i)->at(1) = start->at(1) + (end->at(1) - start->at(1)) * i / size;
    }
    traj.copy_onto_back(end);
  };

  prm_spec.M = 200; // number of prm vertices
  prm_spec.k = 4; // number of prm vertices
  prm_spec.r = 0.15; // number of prm vertices

  space_point_t test_start_state = ss->make_point();
  ss->copy_to(test_start_state);
  
  prm_spec.sample_state = [&](space_point_t& s) {
    // Sample a state from the state space.
    
    s->at(0) = uniform_random(env_xlim[0], env_xlim[1]);
    s->at(1) = uniform_random(env_ylim[0], env_ylim[1]);
    for (int i = 2; i < ss->get_dimension(); i++)
    {
      s->at(i) = test_start_state->at(i);
    }
  };

  prm_query_t prm_query(ss, cs);

  prm_query.get_visualization = visualize_tree;
  prm_query.goal_state = context.first->get_state_space()->make_point();
  prm_query.start_state = context.first->get_state_space()->make_point();
  // prm_query.goal_region_radius = goal_region_radius;

  ss->copy_to(prm_query.start_state);
  for (int i = 0; i < 2; i++)
  {
    prm_query.goal_state->at(i) = goal_vec[i];
  }

  prm_planner.link_and_setup_spec(&prm_spec);
  prm_planner.preprocess();
  prm_planner.link_and_setup_query(&prm_query);
  prm_planner.resolve_query(&checker);
  prm_planner.fulfill_query();



}
