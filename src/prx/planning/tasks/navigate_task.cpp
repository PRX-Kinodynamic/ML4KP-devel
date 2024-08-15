#include "prx/planning/tasks/navigate_task.hpp"

namespace prx
{
navigate_task_t::navigate_task_t(param_loader params, simulation_context planning_context, std::vector<double> goal_vec,
                                 double goal_region_radius)
{
  this->params = params;
  context = planning_context;
  this->goal_vec = goal_vec;

  // robot sim indices
  pos_indices = { 0, 1 };
  vel_indices = { 23, 24 };

  // set bounds for planner -> currently full bounded environment
  // TODO: bring from some config for robot's sim indices.
  std::vector<double> env_xlim = params["env_xlim"].as<std::vector<double>>();
  std::vector<double> env_ylim = params["env_ylim"].as<std::vector<double>>();
  _set_bounds(env_xlim, env_ylim);

  _prepare_specification(goal_vec);
  _prepare_query(goal_vec, goal_region_radius);
  _set_condition_checker();
}

void navigate_task_t::_prepare_specification(std::vector<double> goal_vec)
{
  auto system_group = context.first;
  auto ss = system_group->get_state_space();

  spec = std::make_shared<dirt_specification_t>(context.first, context.second);

  spec->blossom_number = params["blossom"].as<int>();
  spec->use_pruning = params["use_pruning"].as<bool>();

  spec->min_control_steps = params["min_control_steps"].as<double>() * (1.0 / simulation_step);
  spec->max_control_steps = params["max_control_steps"].as<double>() * (1.0 / simulation_step);

  spec->distance_function = [&](const space_point_t& a, const space_point_t& b) {
    return std::sqrt((b->at(pos_indices[0]) - a->at(pos_indices[0])) * (b->at(pos_indices[0]) - a->at(pos_indices[0])) +
                     (b->at(pos_indices[1]) - a->at(pos_indices[1])) * (b->at(pos_indices[1]) - a->at(pos_indices[1])));
  };

  double max_vel = params["max_vel"].as<double>();
  spec->h = [&, max_vel](const space_point_t& s, const space_point_t& s2) {
    return spec->distance_function(s, s2) / max_vel;
  };

  spec->bnb = params["bnb"].as<bool>();

  // spec->sample_state = [ss, goal_vec](space_point_t& state) {
  //   default_sample_state(state, ss);
  //   if (uniform_random(0, 1) < 0.2)
  //   {
  //     state->at(0) = goal_vec[0];
  //     state->at(1) = goal_vec[1];
  //   }
  // };

  // TODO: add spec->expand here for bang bang controls.
}

void navigate_task_t::_prepare_query(std::vector<double> goal_vec, double goal_region_radius)
{
  auto system_group = context.first;
  auto ss = system_group->get_state_space();
  auto cs = system_group->get_control_space();

  query = std::make_shared<dirt_query_t>(ss, cs);

  query->get_visualization = params["visualize_tree"].as<bool>();
  query->goal_region_radius = params["goal_pos_region_radius"].as<double>();

  query->goal_region_radius = goal_region_radius;  // params["goal_pos_region_radius"].as<double>();
  // goal_pos_tolerance = params["goal_pos_region_radius"].as<double>();
  // goal_vel_tolerance = params["goal_vel_region_radius"].as<double>();

  query->start_state = ss->make_point();
  ss->copy_to(query->start_state);

  query->goal_state = ss->make_point();

  for (auto i : pos_indices)
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

  query->goal_check = [&](const space_point_t& point) {
    return goal_distance_function(point, query->goal_state, pos_indices) < query->goal_region_radius;
    //  &&
    //        goal_distance_function(point, query->goal_state, vel_indices) < goal_vel_tolerance;
  };

  // return true;
  param_loader ics = params["ics"];
  spec->valid_check = [&, ics, system_group, cs, ss](trajectory_t& traj) {
    bool valid = default_valid_trajectory(traj, spec->valid_state);
    // bool ics_valid = false;
    if (valid)
    {
      bool do_ics_check = ics["do_ics_check"].as<bool>();
      if (do_ics_check)
      {
        auto final_point = traj.back();
        if (query->goal_check(final_point))
        {
          if (ics["type"].as<std::string>() == "blossom")
          {
            int ics_blossom = 25;
            if (ics.exists("blossom"))
            {
              ics_blossom = ics["blossom"].as<int>();
            }
            std::vector<trajectory_t*> trajs_ics;
            std::vector<plan_t*> plans_ics;
            spec->expand(final_point, plans_ics, trajs_ics, ics_blossom, false);
            for (int i = 0; i < plans_ics.size(); i++)
            {
              // plan_t plan(*plans_ics[i]);
              // trajectory_t traj(*trajs_ics[i]);
              if (default_valid_trajectory(*trajs_ics[i], spec->valid_state))
              {
                return true;
              }
            }

            return false;
          }
          else if (ics["type"].as<std::string>() == "bang-bang")
          {
            double duration = 0.5;
            if (ics.exists("duration"))
            {
              duration = ics["duration"].as<double>();
            }

            std::vector<std::vector<double>> control_list = { { -1.0, 1.0 }, { 1.0, 1.0 }, { -1.0, -1.0 },
                                                              { 1.0, -1.0 }, { 0.0, 1.0 }, { 0.0, -1.0 },
                                                              { -1.0, 0.0 }, { 1.0, 0.0 } };

            trajectory_t traj_ics(ss);
            plan_t plan_ics(cs);

            plan_ics.append_onto_back(duration);
            for (unsigned i = 0; i < control_list.size(); i++)
            {
              traj_ics.clear();
              // plan_ics.clear();
              // spec->sample_plan(plan_ics, final_point);
              // default_sample_plan(plan_ics, cs, spec->max_control_steps, spec->max_control_steps);
              // TODO: What is the correct time duration for bang-bang control.

              cs->copy(plan_ics.back().control, control_list[i]);
              // std::cout << plan_ics.back().control << std::endl;
              spec->propagate(final_point, plan_ics, traj_ics);
              if (default_valid_trajectory(traj_ics, spec->valid_state))
              {
                return true;
              }
            }
            return false;
          }
        }
      }
    }
    return valid;
  };
}

void navigate_task_t::_set_condition_checker()
{
  condition_checker = std::make_shared<condition_check_t>(params["condition_checker"]["type"].as<>(),
                                                          params["condition_checker"]["value"].as<int>());

  if (params["condition_checker"]["first_solution"].as<bool>())
  {
    auto system_group = context.first;
    auto ss = system_group->get_state_space();

    custom_check_t custom_checker = [&, ss]() {
      auto pt1 = ss->make_point();
      ss->copy_to(pt1);
      return query->goal_check(pt1);
    };

    custom_conditions.push_back(std::make_shared<condition_check_t>(custom_checker));

    for (auto cond : custom_conditions)
    {
      condition_checker->add_condition(cond.get());
    }
  }
}

void navigate_task_t::_set_bounds(std::vector<double> env_xlim, std::vector<double> env_ylim)
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

dirt_specification_t* navigate_task_t::get_specification()
{
  return spec.get();
}

dirt_query_t* navigate_task_t::get_query()
{
  return query.get();
}

plan_t navigate_task_t::get_solution_plan()
{
  return query->solution_plan;
}

condition_check_t* navigate_task_t::get_condition_checker()
{
  return condition_checker.get();
}
}  // namespace prx