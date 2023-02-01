#ifndef TORCH_NOT_BUILT
#pragma once
#include "prx/utilities/defs.hpp"
#include "prx/utilities/learned_modules/learned_modules_utils.hpp"
#include "prx/planning/planner_functions/planner_functions.hpp"
#include "prx/planning/planners/dirt.hpp"

#include <torch/torch.h>
#include <torch/script.h>

using namespace prx;

class learned_controller_t
{
    private:
        torch::jit::script::Module controller;
    protected:
        bool normalize_input, delta_input, debug_controller;
        double control_duration, max_duration;
        std::vector<double> state_upper_bounds, state_lower_bounds, control_upper_bounds, control_lower_bounds;
        std::vector<int> state_indices, goal_indices;
    public:
    learned_controller_t(param_loader params)
    {
        std::string controller_file = params["/learned_controller/controller_path"].as<std::string>();
        int random_seed = params["/learned_controller/random_seed"].as<int>();
        torch::manual_seed(random_seed);
        torch::Device device(torch::kCPU);
        torch::NoGradGuard no_grad;

        // Get some controller parameters.
        normalize_input = params["/learned_controller/normalize_input"].as<bool>();
        delta_input = params["/learned_controller/delta_input"].as<bool>();
        control_duration = params["/learned_controller/control_duration"].as<double>();
        max_duration = params["/learned_controller/max_duration"].as<double>();
        debug_controller = params["/learned_controller/debug_controller"].as<bool>();

        state_lower_bounds = params["/plant/state_space_lower_bound"].as<std::vector<double>>();
        state_upper_bounds = params["/plant/state_space_upper_bound"].as<std::vector<double>>();

        control_lower_bounds = params["/plant/control_space_lower_bound"].as<std::vector<double>>();
        control_upper_bounds = params["/plant/control_space_upper_bound"].as<std::vector<double>>();

        state_indices = params["/learned_controller/state_indices"].as<std::vector<int>>();
        goal_indices = params["/learned_controller/goal_indices"].as<std::vector<int>>();
        
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

    double get_control_duration()
    {
        return control_duration;
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
            normalized_state = extract_state(normalize_vector(state,state_lower_bounds,state_upper_bounds),state_indices);
        }
        else
        {
            normalized_state = extract_state(state,state_indices);
        }
        long long input_size = normalized_state.size();
        at::Tensor input = torch::zeros({1,input_size},device);
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

    std::vector<std::vector<double>> get_controls(const std::vector<std::vector<double>>& states, const std::vector<std::vector<double>>& goals)
    {
        /*
            Get multiple predictions from the network.
        */
        torch::Device device(torch::kCPU);
        std::vector<torch::jit::IValue> inputs;
        std::vector<std::vector<double>> normalized_states, normalized_goals;

        if (normalize_input)
        {
            for (int i = 0; i < states.size(); i++)
            {
                normalized_states.push_back(extract_state(normalize_vector(states[i],state_lower_bounds,state_upper_bounds),state_indices));
                normalized_goals.push_back(extract_state(normalize_vector(goals[i],state_lower_bounds,state_upper_bounds),goal_indices));
            }
        }
        else
        {
            for (int i = 0; i < states.size(); i++)
            {
                normalized_states.push_back(extract_state(states[i],state_indices));
                normalized_goals.push_back(extract_state(goals[i],goal_indices));
            }
        }
        if (delta_input)
        {
            for (int i = 0; i < normalized_goals.size(); i++)
            {
                for (int j = 0; j < 2; j++)
                {
                    normalized_goals[i][j] = normalized_goals[i][j] - normalized_states[i][j];
                    normalized_states[i][j] = 0;
                }
            }
        }
        for (int i = 0; i < normalized_states.size(); i++)
        {
            normalized_states[i].insert(normalized_states[i].end(),normalized_goals[i].begin(),normalized_goals[i].end());
        }
        long long input_size_0 = normalized_states.size();
        long long input_size_1 = normalized_states[0].size();
        at::Tensor input = torch::zeros({input_size_0,input_size_1},device);
        for (int i = 0; i < normalized_states.size(); i++)
        {
            for (int j = 0; j < normalized_states[i].size(); j++)
            {
                input[i][j] = normalized_states[i][j];
            }
        }
        inputs.push_back(input);
        auto output = controller.forward(inputs).toTensor();
        std::vector<std::vector<double>> controls;
        for(int i = 0; i < output.size(0); i++)
        {
            std::vector<double> control;
            for(int j = 0; j < output.size(1); j++)
            {
                control.push_back(output[i][j].item().toDouble());
            }
            controls.push_back(denormalize_control(control,control_lower_bounds,control_upper_bounds));
        }
        return controls;
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
            normalized_state = extract_state(normalize_vector(state,state_lower_bounds,state_upper_bounds),state_indices);
            normalized_goal = extract_state(normalize_vector(goal,state_lower_bounds,state_upper_bounds),goal_indices);
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
        if (debug_controller)
        {
            std::cout << "Normalized state: ";
            for (int i = 0; i < normalized_state.size(); i++)
            {
                std::cout << normalized_state[i] << " ";
            }
            std::cout << std::endl;
        }
        long long input_size = normalized_state.size();
        at::Tensor input = torch::zeros({1,input_size},device);
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
    
    void fulfill_query(planner_query_t& query, std::shared_ptr<system_group_t> sg)
    {
        // @TODO for Aravind: Adapt this for the non-goal-reaching case.
        query.solution_plan.clear();
        query.solution_traj.clear();
        double time_so_far = 0;

        std::vector<double> state_vec, goal_vec;
        trajectory_t step_traj(sg -> get_state_space());
        space_point_t current = sg -> get_state_space() -> clone_point(query.start_state);
        sg -> get_state_space() -> copy_vector_from_point(goal_vec,query.goal_state);

        while (time_so_far < max_duration && !query.goal_check(current))
        {
            state_vec.clear();
            step_traj.clear();
            query.solution_plan.clear();
            query.solution_plan.append_onto_back(control_duration);
            sg -> get_state_space() -> copy_vector_from_point(state_vec,current);
            sg -> get_control_space() -> copy_point_from_vector(query.solution_plan.back().control,get_control(state_vec,goal_vec));
            if (debug_controller) std::cout << sg -> get_control_space() -> print_point(query.solution_plan.back().control,4) << " " << query.solution_plan.back().duration << std::endl;
            sg -> propagate(current, query.solution_plan, step_traj);
            if (debug_controller) std::cout << sg -> get_state_space() -> print_point(step_traj.back(),4) << std::endl;
            sg -> get_state_space() -> copy_point(current,step_traj.back());
            for (unsigned i = 0; i < step_traj.size() - 1; i++)
            {
                query.solution_traj.copy_onto_back(step_traj[i]);
            }
            time_so_far += control_duration;
            query.solution_cost += control_duration;
        }
        query.solution_traj.copy_onto_back(current);
        /*
        if (!query.goal_check(current))
        {
            PRX_DEBUG_PRINT
            query.clear_outputs();
        }
        else
        {
            PRX_DEBUG_PRINT
        }
        */
    }

    void fulfill_query(planner_query_t& query, rrt_specification_t& spec)
    {
        // @TODO for Aravind: Adapt this for the non-goal-reaching case.
        query.solution_plan.clear();
        query.solution_traj.clear();
        double time_so_far = 0;

        std::vector<double> state_vec, goal_vec;
        trajectory_t step_traj(spec.state_space);
        plan_t step_plan(spec.control_space);
        space_point_t current = spec.state_space -> clone_point(query.start_state);
        spec.state_space -> copy_vector_from_point(goal_vec,query.goal_state);

        while (time_so_far < max_duration && !query.goal_check(current))
        {
            state_vec.clear();
            step_traj.clear();
            step_plan.clear();
            // query.solution_plan.clear();
            query.solution_plan.append_onto_back(control_duration);
            step_plan.append_onto_back(control_duration);
            spec.state_space -> copy_vector_from_point(state_vec,current);
            spec.control_space -> copy_point_from_vector(query.solution_plan.back().control,get_control(state_vec,goal_vec));
            spec.control_space -> copy_point(step_plan.back().control,query.solution_plan.back().control);
            if (debug_controller) std::cout << spec.control_space -> print_point(query.solution_plan.back().control,4) << " " << query.solution_plan.back().duration << std::endl;
            spec.propagate(current, step_plan, step_traj);
            if (debug_controller) std::cout << spec.state_space -> print_point(step_traj.back(),4) << std::endl;
            spec.state_space -> copy_point(current,step_traj.back());
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
        if (!query.goal_check(current)) query.clear_outputs();
    }

};
#else
#endif
