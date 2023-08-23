#ifndef TORCH_NOT_BUILT
#pragma once

#include "torch/torch.h"
#include "torch/script.h"

#include "prx/simulation/controller.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/planning/planners/rrt.hpp"

namespace prx
{
std::vector<double> extract_state(const space_point_t& full_state, const std::vector<int>& indices)
{
  std::vector<double> state;
  for (auto i : indices)
  {
    state.push_back(full_state->at(i));
  }
  return state;
}

std::vector<double> extract_state_with_quat(const space_point_t& full_state)
{
  std::vector<double> state;
  state.push_back(full_state->at(0));
  state.push_back(full_state->at(1));

  prx::quaternion_t quat = prx::quaternion_t(full_state->at(3), full_state->at(4), full_state->at(5), full_state->at(6));
  auto euler = quat.toRotationMatrix().eulerAngles(0, 1, 2);
  state.push_back(euler(2));

  return state;
}

std::vector<double> denormalize_control(const std::vector<double>& control, const std::vector<double>& upper,
                                        const std::vector<double>& lower)
{
  std::vector<double> denormalized_control;
  for (int i = 0; i < control.size(); i++)
  {
    denormalized_control.push_back(0.5 * (control[i] + 1.0) * (upper[i] - lower[i]) + lower[i]);
  }
  return denormalized_control;
}

class learned_controller_t : public controller_t
{
private:
  torch::jit::script::Module controller;
  system_ptr_t plant;

protected:
  bool delta_input, goal_uses_quat;
  double control_duration, max_duration;
  std::vector<int> state_indices, goal_indices;

public:
  learned_controller_t(const system_ptr_t _plant, param_loader params) : controller_t(_plant, "learned_controller")
  {
    goal = _plant->get_state_space()->make_point();

    std::string controller_path = params["controller_path"].as<std::string>();
    int random_seed = params["random_seed"].as<int>();
    delta_input = params["delta_input"].as<bool>();
    goal_uses_quat = params["goal_uses_quat"].as<bool>();
    control_duration = params["control_duration"].as<double>();
    max_duration = params["max_duration"].as<double>();

    state_indices = params["state_indices"].as<std::vector<int>>();
    goal_indices = params["goal_indices"].as<std::vector<int>>();

    torch::manual_seed(random_seed);
    torch::Device device(torch::kCPU);
    torch::set_num_threads(1);
    torch::NoGradGuard no_grad;

    try
    {
      std::cout << "Loading controller from: " << input_path + controller_path << std::endl;
      controller = torch::jit::load(input_path + controller_path, device);
    }
    catch (const c10::Error& e)
    {
      prx_assert(false, "Error loading the controller: " << e.what());
    }
  }

  void set_goal(space_point_t _goal) override
  {
    get_state_space()->copy_point(goal, _goal);
  }

  using controller_t::compute_controls;
  void compute_controls() override
  {
    space_point_t current = get_state_space()->make_point();
    get_state_space()->copy_to(current);
    std::vector<double> control = get_control(current, goal);
    get_control_space()->copy_from_vector(control);
  }

  std::vector<double> get_control(const space_point_t& state, const space_point_t& goal)
  {
    torch::Device device(torch::kCPU);
    std::vector<torch::jit::IValue> inputs;
    std::vector<double> state_input_vector, goal_input_vector;

    state_input_vector = extract_state(state, state_indices);
    if (goal_uses_quat)
    {
      goal_input_vector = extract_state(goal, goal_indices);
    }
    else
    {
      goal_input_vector = extract_state_with_quat(goal);
    }

    if (delta_input)
    {
      for (int i = 0; i < 2; i++)
      {
        goal_input_vector[i] -= state_input_vector[i];
        state_input_vector[i] = 0;
      }
    }

    state_input_vector.insert(state_input_vector.end(), goal_input_vector.begin(), goal_input_vector.end());
    std::size_t input_size = state_input_vector.size();
    at::Tensor input_tensor = torch::zeros({ 1, input_size }, device);
    for (int i = 0; i < input_size; i++)
    {
      input_tensor[0][i] = state_input_vector[i];
    }
    // at::Tensor input_tensor = torch::from_blob(state_input_vector.data(), { 1, input_size }, device);

    inputs.push_back(input_tensor);

    auto output = controller.forward(inputs).toTensor();
    std::vector<double> control;
    for (int i = 0; i < output.size(1); i++)
    {
      control.push_back(output[0][i].item<double>());
    }
    control =
        denormalize_control(control, get_control_space()->get_upper_bounds(), get_control_space()->get_lower_bounds());

    return control;
  }

  std::vector<std::vector<double>> get_controls(const std::vector<space_point_t>& states,
                                                const std::vector<space_point_t>& goals)
  {
    torch::Device device(torch::kCPU);
    std::vector<torch::jit::IValue> inputs;
    std::vector<std::vector<double>> state_input_vector, goal_input_vector;

    for (int i = 0; i < states.size(); i++)
    {
      state_input_vector.push_back(extract_state(states[i], state_indices));
      if (goal_uses_quat)
      {
        goal_input_vector.push_back(extract_state(goals[i], goal_indices));
      }
      else
      {
        goal_input_vector.push_back(extract_state_with_quat(goals[i]));
      }
    }

    if (delta_input)
    {
      for (int i = 0; i < states.size(); i++)
      {
        for (int j = 0; j < 2; j++)
        {
          goal_input_vector[i][j] -= state_input_vector[i][j];
          state_input_vector[i][j] = 0;
        }
      }
    }

    for (int i = 0; i < states.size(); i++)
    {
      state_input_vector[i].insert(state_input_vector[i].end(), goal_input_vector[i].begin(),
                                   goal_input_vector[i].end());
    }

    std::size_t input_size_0 = state_input_vector.size();
    std::size_t input_size_1 = state_input_vector[0].size();
    at::Tensor input_tensor = torch::zeros({ input_size_0, input_size_1 }, device);
    for (int i = 0; i < input_size_0; i++)
    {
      // input_tensor[i] = torch::from_blob(state_input_vector[i].data(), { input_size_1 }, device);
      for (int j = 0; j < input_size_1; j++)
      {
        input_tensor[i][j] = state_input_vector[i][j];
      }
    }
    inputs.push_back(input_tensor);

    auto output = controller.forward(inputs).toTensor();
    std::vector<std::vector<double>> controls;
    for (int i = 0; i < output.size(0); i++)
    {
      std::vector<double> control;
      for (int j = 0; j < output.size(1); j++)
      {
        control.push_back(output[i][j].item<double>());
      }
      control = denormalize_control(control, get_control_space()->get_upper_bounds(),
                                    get_control_space()->get_lower_bounds());
      controls.push_back(control);
    }

    return controls;
  }

  void fulfill_query(planner_query_t& query, rrt_specification_t& spec)
  {
    query.solution_plan.clear();
    query.solution_traj.clear();
    double time_so_far = 0.;

    std::vector<double> state_vec, goal_vec;
    trajectory_t step_traj(get_state_space());
    plan_t step_plan(get_control_space());
    space_point_t current = get_state_space()->clone_point(query.start_state);

    while (time_so_far < max_duration && !query.goal_check(current))
    {
      state_vec.clear();
      step_traj.clear();
      step_plan.clear();

      query.solution_plan.append_onto_back(control_duration);
      step_plan.append_onto_back(control_duration);

      get_control_space()->copy_point_from_vector(step_plan.back().control, get_control(current, query.goal_state));
      get_control_space()->copy_point(query.solution_plan.back().control, step_plan.back().control);
      spec.propagate(current, step_plan, step_traj);

      get_state_space()->copy_point(current, step_traj.back());
      for (unsigned i = 0; i < step_traj.size() - 1; i++)
      {
        query.solution_traj.copy_onto_back(step_traj[i]);
      }
      time_so_far += control_duration;
      query.solution_cost += control_duration;

      if (!spec.valid_check(step_traj))
      {
        query.clear_outputs();
        return;
      }
    }
    query.solution_traj.copy_onto_back(current);
    if (!query.goal_check(current))
    {
      query.clear_outputs();
    }
  }
};
}  // namespace prx

#else
#endif