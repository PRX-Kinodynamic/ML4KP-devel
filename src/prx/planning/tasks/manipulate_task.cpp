#include "prx/planning/tasks/manipulate_task.hpp"

namespace prx
{
manipulate_task_t::manipulate_task_t(param_loader params, simulation_context planning_context, std::vector<double> goal_vec, std::vector<unsigned> pos_indices, std::vector<unsigned> angle_indices)
{
  this->params = params;
  context = planning_context;

  // object sim indices
  // TODO: bring from some config for object's sim indices.
  robot_vel_indices = { 9, 10 };
  this->pos_indices = pos_indices;
  this->angle_indices = angle_indices;

  // set bounds for planner -> currently full bounded environment
  
  std::vector<double> env_xlim = params["env_xlim"].as<std::vector<double>>();
  std::vector<double> env_ylim = params["env_ylim"].as<std::vector<double>>();
  _set_bounds(env_xlim, env_ylim);
  
  
  _prepare_specification();
  _prepare_query(goal_vec);
}

void manipulate_task_t::_prepare_specification()
{
  spec = std::make_shared<dirt_specification_t>(context.first, context.second);

  spec->use_pruning = params["use_pruning"].as<bool>();
  spec->blossom_number = params["blossom"].as<int>();

  spec->min_control_steps = params["min_control_steps"].as<double>() * (1.0 / simulation_step);
  spec->max_control_steps = params["max_control_steps"].as<double>() * (1.0 / simulation_step);

  spec->distance_function = [&](const space_point_t& a, const space_point_t& b) {
    return std::sqrt((b->at(pos_indices[0]) - a->at(pos_indices[0])) * (b->at(pos_indices[0]) - a->at(pos_indices[0])) + (b->at(pos_indices[1]) - a->at(pos_indices[1])) * (b->at(pos_indices[1]) - a->at(pos_indices[1])));
  };
  

  double max_vel = params["max_vel"].as<double>();
  spec->h = [&](const space_point_t& s, const space_point_t& s2) { return spec->distance_function(s, s2)/max_vel; };

  // TODO: add spec->expand here for bang bang controls.
  // dirt_spec.expand = [&](space_point_t& s, std::vector<plan_t*>& plans, std::vector<trajectory_t*>& trajs, int bn, bool blossom_expand) {
  //   if (blossom_expand)
  //   {
  //     plan_t plan(cs);
  //     plan.append_onto_back(1.0);
  //     for (auto control : control_list)
  //     {
  //       cs->copy(plan.back().control, control);
  //       trajectory_t* traj = new trajectory_t(ss);
  //       dirt_spec.propagate(s, plan, *traj);
  //       plans.push_back(new plan_t(plan));
  //       trajs.push_back(traj);
  //     }
  //   }
  //   else
  //   {
  //     default_expand(s, plans, trajs, bn, sg, dirt_spec.sample_plan, dirt_spec.propagate);
  //   }
  // };
}

void manipulate_task_t::_prepare_query(std::vector<double> goal_vec)
{
  auto system_group = context.first;
  auto ss = system_group->get_state_space();
  auto cs = system_group->get_control_space();

  query = std::make_shared<dirt_query_t>(ss, cs);

  query->get_visualization = params["visualize_tree"].as<bool>();
  query->goal_region_radius = params["goal_pos_region_radius"].as<double>();

  goal_pos_tolerance = params["goal_pos_region_radius"].as<double>();
  goal_angle_tolerance = params["goal_angle_region_radius"].as<double>();
  goal_vel_tolerance = params["goal_vel_region_radius"].as<double>();

  query->start_state = ss->make_point();
  ss->copy_to(query->start_state);

  query->goal_state = ss->make_point();

  for (auto i : pos_indices)
  {
    query->goal_state->at(i) = goal_vec[i];
  }

  for (auto i : angle_indices)
  {
    query->goal_state->at(i) = goal_vec[i];
  }

  goal_distance_function = [&](const space_point_t& a, const space_point_t& b, std::vector<unsigned> indices) {
    double sqr_distance = 0;
    for (auto i : indices)
    {
      sqr_distance += (a->at(i) - b->at(i)) * (a->at(i) - b->at(i));
    }
    return std::sqrt(sqr_distance);
  };

  // for (auto i : pos_indices)
  // {
  //   std::cout << "current: " << query->start_state->at(i) << std::endl;
  //   std::cout << "Goal state: " << query->goal_state->at(i) << std::endl;
  // }
  
  // for (auto i : angle_indices)
  // {
  //   std::cout << "current: " << query->start_state->at(i) << std::endl;
  //   std::cout << "Goal state: " << query->goal_state->at(i) << std::endl;
  // }

  quaternion_t goal_q = Eigen::Quaterniond(query->goal_state->at(angle_indices[0]), query->goal_state->at(angle_indices[1]), query->goal_state->at(angle_indices[2]), query->goal_state->at(angle_indices[3]));

  query->goal_check = [&, goal_q](const space_point_t& point) {
    //check position
    bool is_position = goal_distance_function(point, query->goal_state, pos_indices) < goal_pos_tolerance;
    //check angle
    quaternion_t curr_q = Eigen::Quaterniond(point->at(angle_indices[0]), point->at(angle_indices[1]), point->at(angle_indices[2]), point->at(angle_indices[3]));
    double angular_diff = curr_q.angularDistance(goal_q);
    
    // std::cout << "angular diff: " << angular_diff << std::endl;
    // angular_diff *= angular_diff;
    bool is_angle = angular_diff < goal_angle_tolerance;


    // robot velocity -> zero
    // bool is_vel = goal_distance_function(point, query->goal_state, robot_vel_indices) < goal_vel_tolerance;

    return is_position && is_angle;
  };
}

void manipulate_task_t::_set_bounds(std::vector<double> env_xlim, std::vector<double> env_ylim)
{
  auto ss = this->context.first->get_state_space();
  std::vector<double> ub = ss->get_upper_bounds();
  std::vector<double> lb = ss->get_lower_bounds();

  lb.at(0) = env_xlim[0];
  lb.at(1) = env_ylim[0];
  ub.at(0) = env_xlim[1];
  ub.at(1) = env_ylim[1];

  ss->set_bounds(lb, ub);
}

dirt_specification_t* manipulate_task_t::get_specification()
{
  return spec.get();
}

dirt_query_t* manipulate_task_t::get_query()
{
  return query.get();
}

plan_t manipulate_task_t::get_solution_plan(){
    return query->solution_plan;
}

}  // namespace prx
