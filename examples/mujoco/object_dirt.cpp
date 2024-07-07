// #ifndef TORCH_NOT_BUILT
#include "prx/utilities/defs.hpp"
#include "prx/mujoco/mj_simulator.hpp"
#include "prx/planning/planners/dirt.hpp"

#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>

using namespace boost::filesystem;

using namespace prx;

int main(int argc, char* argv[])
{
  param_loader params;

  params = param_loader("examples/JIST/object_dirt.yaml");
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
  auto sg = context.first;
  auto ss = context.first->get_state_space();
  auto cs = context.first->get_control_space();

  for (int i = 0; i < 5; i++)
  {
    sim->step_simulation();
  }

  dirt_t dirt(params["planner_name"].as<>());
  dirt_specification_t dirt_spec(context.first, context.second);

  float min_control_scaling = 0.1;
  float max_control_scaling = 1.0;
  dirt_spec.min_control_steps = min_control_scaling * (1.0 / simulation_step);
  dirt_spec.max_control_steps = max_control_scaling * (1.0 / simulation_step);
  dirt_spec.use_pruning = params["use_pruning"].as<bool>();

  dirt_spec.distance_function = [](const space_point_t& a, const space_point_t& b) {
    return std::sqrt((b->at(0) - a->at(0)) * (b->at(0) - a->at(0)) + (b->at(1) - a->at(1)) * (b->at(1) - a->at(1)));
  };

  double max_vel = params["random_seed"].as<double>();
  dirt_spec.blossom_number = params["blossom"].as<int>();
  dirt_spec.h = [&](const space_point_t& s, const space_point_t& s2) {
    return 0; // dirt_spec.distance_function(s, s2) / max_vel;
  };

  std::vector<std::vector<double>> control_list = { { -1., -1. }, { -1., 0. }, { -1., 1. }, { 0., -1. },
                                                    { 0., 1. },   { 1., -1. }, { 1., 0. },  { 1., 1. } };
  dirt_spec.expand = [&](space_point_t& s, std::vector<plan_t*>& plans, std::vector<trajectory_t*>& trajs, int bn,
                         bool blossom_expand) {
    // if (blossom_expand)
    // {
    //   plan_t plan(cs);
    //   plan.append_onto_back(1.0);
    //   for (auto control : control_list)
    //   {
    //     cs->copy(plan.back().control, control);
    //     trajectory_t* traj = new trajectory_t(ss);
    //     dirt_spec.propagate(s, plan, *traj);
    //     plans.push_back(new plan_t(plan));
    //     trajs.push_back(traj);
    //   }
    // }
    // else
    // {
      default_expand(s, plans, trajs, bn, sg, dirt_spec.sample_plan, dirt_spec.propagate);
    // }
  };

  // dirt query
  dirt_query_t dirt_query(context.first->get_state_space(), context.first->get_control_space());

  dirt_query.get_visualization = params["visualize_tree"].as<bool>();

  dirt_query.goal_state = context.first->get_state_space()->make_point();

  dirt_query.start_state = context.first->get_state_space()->make_point();

  ss->copy_to(dirt_query.start_state);
  std::cout << "#### " << ss->print_point(dirt_query.start_state) << std::endl;

  // goal region radius
  dirt_query.goal_region_radius = params["goal_pos_region_radius"].as<double>();
  auto goal_pos_region_radius = params["goal_pos_region_radius"].as<double>();
  auto goal_angle_region_radius = params["goal_angle_region_radius"].as<double>();

  std::vector<double> goal_pos_vec = params["goal_pos"].as<std::vector<double>>();
  std::vector<double> goal_angle_vec = params["goal_angle"].as<std::vector<double>>();
  std::vector<double> env_xlim = params["env_xlim"].as<std::vector<double>>();
  std::vector<double> env_ylim = params["env_ylim"].as<std::vector<double>>();

  auto ub = ss->get_upper_bounds();
  auto lb = ss->get_lower_bounds();

  lb.at(0) = env_xlim[0];
  lb.at(1) = env_ylim[0];
  ub.at(0) = env_xlim[1];
  ub.at(1) = env_ylim[1];
  ss->set_bounds(lb, ub);

  Eigen::Quaterniond q;
  q = Eigen::AngleAxisd(goal_angle_vec[2], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(goal_angle_vec[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(goal_angle_vec[0], Eigen::Vector3d::UnitX());
  std::cout << "#### " << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << std::endl;

  dirt_query.goal_state->at(2) = goal_pos_vec.at(0);
  dirt_query.goal_state->at(3) = goal_pos_vec.at(1);
  dirt_query.goal_state->at(5) = q.w();
  dirt_query.goal_state->at(6) = q.x();
  dirt_query.goal_state->at(7) = q.y();
  dirt_query.goal_state->at(8) = q.z();

  // check whether sampled state is in goal region
  dirt_query.goal_check = [&](const space_point_t& point) {
    quaternion_t q;
    q.w() = point->at(5);
    q.x() = point->at(6);
    q.y() = point->at(7);
    q.z() = point->at(8);
    auto e = quaternion_to_euler(q);

    // intialize a vector with the goal pos from point
    std::vector<double> curr_pos = { point->at(2), point->at(3) };

    // sqrt between curr_pos and goal_pos_vec
    double dist = std::sqrt((goal_pos_vec[0] - curr_pos[0]) * (goal_pos_vec[0] - curr_pos[0]) +
                            (goal_pos_vec[1] - curr_pos[1]) * (goal_pos_vec[1] - curr_pos[1]));

    return std::fabs(norm_angle_pi(e[2] - goal_angle_vec[2])) < goal_angle_region_radius &&
           (dist < goal_pos_region_radius);
    // return dirt_spec.distance_function(point, dirt_query.goal_state) < dirt_query.goal_region_radius;
  };

  condition_check_t checker(params["checker_type"].as<>(), params["checker_value"].as<int>());  //

  dirt.link_and_setup_spec(&dirt_spec);

  dirt.preprocess();

  dirt.link_and_setup_query(&dirt_query);

  dirt.resolve_query(&checker);

  dirt.fulfill_query();

  if (dirt_query.solution_traj.size() == 0)
  {
    std::cout << "No solution found!" << std::endl;
  }
  else
  {
    space_point_t achieved_goal = ss->clone_point(dirt_query.solution_traj.back());
    quaternion_t achieved_q;
    achieved_q.w() = achieved_goal->at(5);
    achieved_q.x() = achieved_goal->at(6);
    achieved_q.y() = achieved_goal->at(7);
    achieved_q.z() = achieved_goal->at(8);
    auto e = quaternion_to_euler(q);
    std::cout << "Achieved Euler: " << e[2] << " radians and " << e[2] * 180 / PRX_PI << " degrees" << std::endl;
  }

  unsigned count = 0;
  std::string output_folder = params["output_tree_folder"].as<std::string>();

  for (const auto& entry : directory_iterator(output_folder))
  {
    remove(entry.path());
  }

  for (auto traj : dirt_query.tree_visualization)
  {
    std::string filename = output_folder + std::to_string(count) + ".txt";
    std::ofstream fout(filename);
    fout << traj.print(16);
    count++;
  }

  prx::constants::precision = 16;
  if (params["output_plan"].as<bool>())
  {
    dirt_query.solution_plan.to_file(params["output_plan_file"].as<std::string>());
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