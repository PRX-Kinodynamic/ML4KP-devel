#ifndef TORCH_NOT_BUILT

#pragma once
#include "prx/utilities/defs.hpp"
#include "prx/utilities/learned_modules/learned_modules_utils.hpp"

#include <torch/torch.h>
#include <torch/script.h>

using namespace prx;

class reachability_estimator_t
{
    private:
        torch::jit::script::Module predictor;
        double threshold;
    protected:
        bool normalize_input, use_reachability_estimator;
        std::vector<double> state_upper_bounds, state_lower_bounds;
        std::vector<int> state_indices, goal_indices;
    public:
    reachability_estimator_t(param_loader params)
    {
        use_reachability_estimator = params["use_reachability_estimator"].as<bool>();
        if (use_reachability_estimator)
        {
            std::string predictor_file = params["/reachability_estimator/predictor_path"].as<std::string>();
            int random_seed = params["/reachability_estimator/random_seed"].as<int>();
            torch::manual_seed(random_seed);
            torch::Device device(torch::kCPU);
            torch::NoGradGuard no_grad;
            
            // Get some parameters.
            normalize_input = params["/reachability_estimator/normalize_input"].as<bool>();
            state_lower_bounds = params["/plant/state_space_lower_bound"].as<std::vector<double>>();
            state_upper_bounds = params["/plant/state_space_upper_bound"].as<std::vector<double>>();
            
            state_indices = params["/reachability_estimator/state_indices"].as<std::vector<int>>();
            goal_indices = params["/reachability_estimator/goal_indices"].as<std::vector<int>>();

            threshold = params["/reachability_estimator/threshold"].as<double>();
            
            try
            {
                std::cout << input_path + predictor_file << std::endl;
                predictor = torch::jit::load(input_path+predictor_file,device);
            }
            catch(const c10::Error& e)
            {
                std::cout << e.msg() << std::endl;
                prx_assert(false,"Error loading the network from file!");
            }
        }
    }
    

    double get_estimate(const std::vector<double>& state, const std::vector<double>& goal)
    {
        if (use_reachability_estimator)
        {
            torch::Device device(torch::kCPU);
            std::vector<torch::jit::IValue> inputs;

            std::vector<double> normalized_state, normalized_goal;
            if (normalize_input)
            {
                normalized_state = extract_state(normalize_vector(state, state_lower_bounds, state_upper_bounds),state_indices);
                normalized_goal  = extract_state(normalize_vector(goal, state_lower_bounds, state_upper_bounds),goal_indices);
            }
            else
            {
                normalized_state = extract_state(state,state_indices);
                normalized_goal  = extract_state(goal,goal_indices);
            }
            
            normalized_state.insert(normalized_state.end(),normalized_goal.begin(),normalized_goal.end());
            
            long long input_size = normalized_state.size();
            at::Tensor input = torch::zeros({1,input_size},device);
            for (int i = 0; i < normalized_state.size(); i++)
            {
                input[0][i] = normalized_state[i];
            }
            inputs.push_back(input);
            
            torch::Tensor output_tensor = predictor.forward(inputs).toTensor();
            double output = output_tensor[0].item<double>();
            return sigmoid(output);
        }
        else
        {
            return 1.0;
        }
    }

    bool is_reachable(const std::vector<double>& state, const std::vector<double>& goal)
    {
        double reachability_estimate = get_estimate(state, goal);
        return reachability_estimate > threshold;
    }
};

#else
#endif