#ifndef TORCH_NOT_BUILT
#pragma once
#include "prx/utilities/defs.hpp"
#include "prx/planning/planner_functions/planner_functions.hpp"
#include "prx/planning/planners/rrt.hpp"

#include <torch/torch.h>
#include <torch/script.h>

using namespace prx;

class learned_controller_t
{
    private:
        torch::jit::script::Module controller;
    protected:
        bool normalize_input, delta_input;
        double control_duration;
        std::vector<double> state_upper_bounds, state_lower_bounds, control_upper_bounds, control_lower_bounds;
        std::vector<int> state_indices, goal_indices;
    public:
    learned_controller_t(param_loader params)
    {
        std::string controller_file = params["controller_path"].as<std::string>();
        int random_seed = params["random_seed"].as<int>();
        torch::manual_seed(random_seed);
        torch::Device device(torch::kCPU);

        // Get some controller parameters.
        normalize_input = params["normalize_input"].as<bool>();
        delta_input = params["delta_input"].as<bool>();
        control_duration = params["control_duration"].as<double>();

        state_lower_bounds = params["/plant/state_space_lower_bound"].as<std::vector<double>>();
        state_upper_bounds = params["/plant/state_space_upper_bound"].as<std::vector<double>>();

        control_lower_bounds = params["/plant/control_space_lower_bound"].as<std::vector<double>>();
        control_upper_bounds = params["/plant/control_space_upper_bound"].as<std::vector<double>>();

        state_indices = params["state_indices"].as<std::vector<int>>();
        goal_indices = params["goal_indices"].as<std::vector<int>>();
        
        try
        {
            std::cout << input_path + controller_file << std::endl;
            controller = torch::jit::load(input_path+controller_file,device);
        }
        catch(const c10::Error& e)
        {
            prx_assert(false,"Error loading the network from file!");
        }
    }

    std::vector<double> get_control(const std::vector<double>& state)
    {
        /*
            Gets a single prediction from the network.
        */
        torch::Device device(torch::kCPU);
        std::vector<torch::jit::IValue> inputs;
        std::vector<double> normalized_state;
        if (normalize_input)
        {
            normalized_state = extract_state(normalize_state(state,state_lower_bounds,state_upper_bounds),state_indices);
        }
        else
        {
            normalized_state = extract_state(state,state_indices);
        }
        at::Tensor input = torch::zeros({1,normalized_state.size()},device);
        for (int i = 0; i < normalized_state.size(); i++)
        {
            input[0][i] = normalized_state[i];
        }
        inputs.push_back(input);
        // inputs.push_back(torch::from_blob(normalized_state.data(),{1,normalized_state.size()}).to(torch::kFloat32));
        auto output = controller.forward(inputs).toTensor();
        std::vector<double> control;
        for(int i = 0; i < output.size(1); i++)
        {
            control.push_back(output[0][i].item().toDouble());
        }
        return denormalize_control(control,control_lower_bounds,control_upper_bounds);
    }
    
    std::vector<double> get_control(const std::vector<double>& state, const std::vector<double>& goal)
    {
        /*
            Gets a single prediction from the network.
        */
        torch::Device device(torch::kCPU);
        std::vector<torch::jit::IValue> inputs;
        std::vector<double> normalized_state, normalized_goal;
        if (normalize_input)
        {
            normalized_state = extract_state(normalize_state(state,state_lower_bounds,state_upper_bounds),state_indices);
            normalized_goal = extract_state(normalize_state(goal,state_lower_bounds,state_upper_bounds),goal_indices);
        }
        else
        {
            normalized_state = extract_state(state,state_indices);
            normalized_goal = extract_state(goal,goal_indices);
        }
        if (delta_input)
        {
            for (int i = 0; i < 2; i++)
            {
                normalized_goal[i] = normalized_goal[i] - normalized_state[i];
                normalized_state[i] = 0;
            }
        }
        normalized_state.insert(normalized_state.end(),normalized_goal.begin(),normalized_goal.end());
        for (auto i : normalized_state)
        {
            std::cout << i << " ";
        }
        std::cout << std::endl;
        at::Tensor input = torch::zeros({1,normalized_state.size()},device);
        for (int i = 0; i < normalized_state.size(); i++)
        {
            input[0][i] = normalized_state[i];
        }
        inputs.push_back(input);
        // @aravind: The following lines are supposed to work. But they don't.
        // auto options = torch::TensorOptions().device(torch::kCPU);
        // inputs.push_back(torch::from_blob(normalized_state.data(),{1,normalized_state.size()},options));
        auto output = controller.forward(inputs).toTensor();
        std::vector<double> control;
        for(int i = 0; i < output.size(1); i++)
        {
            control.push_back(output[0][i].item().toDouble());
        }
        return denormalize_control(control,control_lower_bounds,control_upper_bounds);
    }

    std::vector<double> extract_state(const std::vector<double>& state, const std::vector<int>& indices)
    {
        /*
            Extracts the state from the state vector.
        */
        std::vector<double> extracted_state;
        for (int i = 0; i < indices.size(); i++)
        {
            prx_assert(indices[i] < state.size(),"Index out of bounds!");
            extracted_state.push_back(state[indices[i]]);
        }
        return extracted_state;
    }

    std::vector<double> normalize_state(const std::vector<double>& state, const std::vector<double>& lower_bounds, const std::vector<double>& upper_bounds)
    {
        /*
            Normalizes the state to be between 0 and 1.
        */
        std::vector<double> normalized_state;
        for(int i = 0; i < state.size(); i++)
        {
            normalized_state.push_back((state[i] - lower_bounds[i])/(upper_bounds[i] - lower_bounds[i]));
        }
        return normalized_state;
    }

    std::vector<double> denormalize_control(const std::vector<double>& control, const std::vector<double>& lower_bounds, const std::vector<double>& upper_bounds)
    {
        /*
            Denormalizes the control to be between the lower and upper bounds.
            Assumes the network outputs between -1 and 1.
        */
        std::vector<double> denormalized_control;
        for(int i = 0; i < control.size(); i++)
        {
            denormalized_control.push_back(0.5*(control[i] + 1.0)*(upper_bounds[i] - lower_bounds[i]) + lower_bounds[i]);
        }
        return denormalized_control;
    }

    void fulfill_query(planner_query_t& query, std::shared_ptr<system_group_t> sg, int horizon)
    {
        // @TODO for Aravind: Adapt this for the non-goal-reaching case.
        query.solution_plan.clear();
        query.solution_traj.clear();
        double time_so_far = 0;

        std::vector<double> state_vec, goal_vec;
        space_point_t current = sg -> get_state_space() -> clone_point(query.start_state);
        sg -> get_state_space() -> copy_vector_from_point(goal_vec,query.goal_state);

        while (time_so_far < horizon && !query.goal_check(current))
        {
            state_vec.clear();
            query.solution_plan.append_onto_back(control_duration);
            sg -> get_state_space() -> copy_vector_from_point(state_vec,current);
            sg -> get_control_space() -> copy_point_from_vector(query.solution_plan.back().control,get_control(state_vec,goal_vec));
            std::cout << sg -> get_control_space() -> print_point(query.solution_plan.back().control) << std::endl;
            sg -> propagate(query.start_state, query.solution_plan, query.solution_traj);
            sg -> get_state_space() -> copy_point(current,query.solution_traj.back());
            time_so_far += control_duration;
        }
    }
};
#else
#endif
