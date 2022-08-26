#pragma once
#include "prx/utilities/defs.hpp"
#include "prx/utilities/learned_modules/learned_modules_utils.hpp"

#include <torch/torch.h>
#include <torch/script.h>

using namespace prx;

class waypoint_predictor_t
{
    private:
        torch::jit::script::Module predictor;
    protected:
        bool use_waypoint_predictor;
    public:
        std::vector<int> state_indices, goal_indices;
    waypoint_predictor_t(param_loader params)
    {
        // use_waypoint_predictor = params.get_param("use_waypoint_predictor", false);
        use_waypoint_predictor = true;
        if (use_waypoint_predictor)
        {
            std::string predictor_file = params["/waypoint_predictor/predictor_path"].as<std::string>();
            int random_seed = params["random_seed"].as<int>();
            torch::manual_seed(random_seed);
            torch::Device device(torch::kCPU);
            torch::NoGradGuard no_grad;
            
            // Get some parameters.
            // normalize_input = params["/waypoint_predictor/normalize_input"].as<bool>();
            // normalize_output = params["/waypoint_predictor/normalize_output"].as<bool>();
            // state_lower_bounds = params["/plant/state_space_lower_bound"].as<std::vector<double>>();
            // state_upper_bounds = params["/plant/state_space_upper_bound"].as<std::vector<double>>();
            
            state_indices = params["/waypoint_predictor/state_indices"].as<std::vector<int>>();
            goal_indices = params["/waypoint_predictor/goal_indices"].as<std::vector<int>>();

            try
            {
                std::cout << input_path + predictor_file << std::endl;
                predictor = torch::jit::load(input_path+predictor_file,device);
            }
            catch(const c10::Error& e)
            {
                std::cout << e.what() << std::endl;
                prx_assert(false,"Error loading the network from file!");
            }
        }
    }

    std::vector<double> get_waypoint(const std::vector<double>& state, const std::vector<double>& infos, const std::vector<double>& goal)
    {
        if (use_waypoint_predictor)
        {
            torch::Device device(torch::kCPU);
            std::vector<torch::jit::IValue> inputs;

            std::vector<double> normalized_state, normalized_goal;
            // if (normalize_input)
            // {
            //     normalized_state = extract_state(normalize_vector(state, state_lower_bounds, state_upper_bounds),state_indices);
            //     normalized_goal  = extract_state(normalize_vector(goal, state_lower_bounds, state_upper_bounds),goal_indices);
            // }
            // else
            // {
                normalized_state = extract_state(state,state_indices);
                // normalized_goal  = extract_state(goal,goal_indices);
            // }
            normalized_state.insert(normalized_state.end(),infos.begin(),infos.end());
            // normalized_state.insert(normalized_state.end(),normalized_goal.begin(),normalized_goal.end());
            
            at::Tensor input = torch::zeros({1,normalized_state.size()},device);
            
            for (int i = 0; i < normalized_state.size(); i++)
            {
                input[0][i] = normalized_state[i];
            }
            inputs.push_back(input);

            at::Tensor goal_input = torch::zeros({1,goal.size()},device);
            for (int i = 0; i < goal.size(); i++)
            {
                goal_input[0][i] = goal[i];
            }
            inputs.push_back(goal_input);
            
            torch::Tensor output_tensor = predictor.forward(inputs).toTensor();
            std::vector<double> waypoint;
            for (int i = 0; i < output_tensor.size(1); i++)
            {
                waypoint.push_back(output_tensor[0][i].item().toDouble());
            }
            // if (normalize_output)
            // {
            //     waypoint = denormalize_state(waypoint, state_lower_bounds, state_upper_bounds);
            // }
            return waypoint;
        }
        else
        {
            return goal;
        }
    }
    std::vector<double> get_waypoint(const std::vector<double>& state, const std::vector<double>& infos)
    {
        if (use_waypoint_predictor)
        {
            torch::Device device(torch::kCPU);
            std::vector<torch::jit::IValue> inputs;

            std::vector<double> normalized_state;
            // if (normalize_input)
            // {
            //     normalized_state = extract_state(normalize_vector(state, state_lower_bounds, state_upper_bounds),state_indices);
            // }
            // else
            // {
                normalized_state = extract_state(state,state_indices);
            // }
            normalized_state.insert(normalized_state.end(),infos.begin(),infos.end());
            
            at::Tensor input = torch::zeros({1,normalized_state.size()},device);
            
            for (int i = 0; i < normalized_state.size(); i++)
            {
                input[0][i] = normalized_state[i];
            }
            inputs.push_back(input);
            
            torch::Tensor output_tensor = predictor.forward(inputs).toTensor();
            std::vector<double> waypoint;
            for (int i = 0; i < output_tensor.size(1); i++)
            {
                waypoint.push_back(output_tensor[0][i].item().toDouble());
            }
            // if (normalize_output)
            // {
            //     waypoint = denormalize_state(waypoint, state_lower_bounds, state_upper_bounds);
            // }
            return waypoint;
        }
        else
        {
            return state;
        }
    }
};