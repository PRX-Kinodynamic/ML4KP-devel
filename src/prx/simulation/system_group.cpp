
#include "prx/simulation/simulator.hpp"
#include "prx/simulation/system_group.hpp"

#include <algorithm>

namespace prx
{
// system_group_t::system_group_t(const std::vector<system_ptr_t>& sys_group)
system_group_t::system_group_t(const std::vector<system_ptr_t>& sys_group, plant_type p_type)
{
  group = sys_group;
  // construct the state and control spaces after checking for overlap
  std::vector<system_ptr_t> to_remove;
  for (auto g1 : group)
  {
    for (auto g2 : group)
    {
      if (g1 != g2 && (is_child_system(g1, g2) || is_child_system(g2, g1)))
      {
        if (g1->get_pathname().length() > g2->get_pathname().length())
          to_remove.push_back(g1);
        else
          to_remove.push_back(g2);
      }
    }
  }
  auto it = std::unique(to_remove.begin(), to_remove.end());
  to_remove.resize(std::distance(to_remove.begin(), it));
  group.erase(std::remove_if(std::begin(group), std::end(group),
                             [&](system_ptr_t x) {
                               return std::find(std::begin(to_remove), std::end(to_remove), x) != std::end(to_remove);
                             }),
              std::end(group));

  // compose the state spaces and control spaces
  std::vector<const space_t*> state_spaces;
  std::vector<const space_t*> control_spaces;
  std::vector<const space_t*> parameter_spaces;

  for (auto g1 : group)
  {
    state_spaces.push_back(g1->get_state_space());
    control_spaces.push_back(g1->get_control_space());
    parameter_spaces.push_back(g1->get_parameter_space());
  }
  state_space = new space_t(state_spaces);
  control_space = new space_t(control_spaces);
  _parameter_space = new space_t(parameter_spaces);
}

system_group_t::~system_group_t()
{
  delete state_space;
  delete control_space;
  group.clear();
}

void system_group_t::propagate(space_point_t start_state, const plan_t& plan, space_point_t result)
{
  state_space->copy_from_point(start_state);

  for (const plan_step_t& step : plan)
  {
    int steps = (int)((step.duration / simulation_step) + .1);
    // int i = 0;
    if (steps > 0)
    {
      // for( ; i < steps; i++ )
      // {
      // 	if (i == 0) p_step = propagate_step::FIRST_STEP;
      // 	else if (i > 0 && i < steps-1) p_step = propagate_step::MIDDLE_STEP;
      // 	else p_step = propagate_step::FINAL_STEP;

      // 	propagate_once(step.control,p_step);
      // }
      propagate(steps, step.control);
    }
  }
  state_space->copy_to_point(result);
}

void system_group_t::propagate(space_point_t start_state, controller_ptr_t ctrl, condition_check_t& cond_check,
                               space_point_t result)
{
  state_space->copy_from_point(start_state);

  int i = 0;
  do
  {
    ctrl->compute_controls();
    propagate_once(nullptr);
  } while (!cond_check.check());

  state_space->copy_to_point(result);
}

void system_group_t::propagate(space_point_t start_state, controller_ptr_t ctrl, condition_check_t& cond_check,
                               trajectory_t& result)
{
  state_space->copy_from_point(start_state);

  int i = 0;
  do
  {
    ctrl->compute_controls();
    propagate_once(nullptr);
    result.copy_onto_back(state_space);
  } while (!cond_check.check());

  // state_space -> copy_to_point(result);
}

void system_group_t::propagate(space_point_t start_state, const plan_t& plan, trajectory_t& traj)
{
  state_space->copy_from(start_state);

  traj.clear();
  traj.copy_onto_back(state_space);
  for (const plan_step_t& step : plan)
  {
    int steps = (int)((step.duration / simulation_step) + .1);
    // int i = 0;
    if (steps > 0)
    {
      propagate(steps, step.control, &traj);
      // for( ; i < steps; i++ )
      // {
      // 	if (i == 0) p_step = propagate_step::FIRST_STEP;
      // 	else if (i > 0 && i < steps-1) p_step = propagate_step::MIDDLE_STEP;
      // 	else p_step = propagate_step::FINAL_STEP;

      // 	propagate_once(step.control,p_step);
      // 	traj.copy_onto_back(state_space);
      // }
    }
  }
}

void system_group_t::propagate(int steps, space_point_t control, trajectory_t* traj)
{
  for (int i = 0; i < steps; i++)
  {
    propagate_once(control);
    if (traj != nullptr)
    {
      traj->copy_onto_back(state_space);
    }
  }
}

void system_group_t::propagate_once(space_point_t control)
{
  if (control != nullptr)
  {
    control_space->copy_from_point(control);
  }
  for (auto s : group)
  {
    s->compute_control();
    s->get_control_space()->enforce_bounds();
  }
  // for(auto s : group)
  // {
  // 	s->propagate(simulation_step, step);
  // }
  sim->step_simulation();
}

void system_group_t::compute_stopping_maneuver(space_point_t start_state, double& time)
	{
		prx_assert(group.size() == 1, "[system_group_t::compute_stopping_maneuver] Expected group of size 1 but got "<<group.size());
		group[0]->compute_stopping_maneuver(start_state, time);
	}

void system_group_t::steer(space_point_t x_new, const space_point_t x_nearest, const space_point_t x_rand,
                           const double eta, distance_function_t distance_function)
{
  const double total_steps{ eta / prx::simulation_step };
  std::vector<double> steps{ prx::linspace(0.0, 1.0, total_steps) };

  state_space->copy_from(x_nearest);
  for (auto ti : steps)
  {
    steer_once(x_nearest, x_rand, ti);
    state_space->copy_to(x_new);
    if (eta < distance_function(x_nearest, x_new))
      break;
  }
  state_space->copy_to(x_new);
}

void system_group_t::steer(trajectory_t& traj_out, const space_point_t x_nearest, const space_point_t x_rand,
                           const double max_dist, distance_function_t distance_function)
{
  const double dist_to_x_rand{ distance_function(x_nearest, x_rand) };
  const double dist{ std::min(max_dist, dist_to_x_rand) };
  const double total_steps{ dist / prx::simulation_step };
  const double t_end{ dist / dist_to_x_rand };
  std::vector<double> steps{ prx::linspace(0.0, t_end, total_steps) };
  // PRX_DEBUG_VAR_3(dist_to_x_rand, max_dist, dist);
  // PRX_DEBUG_VAR_3(t_end, total_steps, steps.back());
  // state_space->copy_from(x_nearest);
  for (auto ti : steps)
  {
    steer_once(x_nearest, x_rand, ti);
    traj_out.copy_onto_back(state_space);
  }
  traj_out.copy_onto_back(state_space);
}

void system_group_t::steer_once(const space_point_t x0, const space_point_t x1, const double ti)
{
  for (auto s : group)
  {
    // TODO: This won't work with multiple systems, need to split the x_rand
    s->steer(x0, x1, ti);
  }
}

}  // namespace prx
