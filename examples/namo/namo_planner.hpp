#pragma once

#include "environment.hpp"
#include "push_controller.hpp"
#include "wavefront_planner.hpp"
#include <unordered_map>
#include <unordered_set>

namespace prx {

// Helper function for character conversion - MOVED HERE BEFORE THE CLASS
inline int convert_ch_to_int(char ch) {
    if (ch >= 'a' && ch <= 'z') {
        return ch - 'a';
    }
    else if (ch >= 'A' && ch <= 'Z') {
        return ch - 'A' + 26;
    }
    else{
        return ch - '0' + 52;
    }
}

/**
 * @brief Handles action sequence optimization for NAMO planning
 */
class NAMOPlanner {
public:
    /**
     * @brief Constructor for NAMOPlanner
     * 
     * @param env Reference to the environment
     * @param controller Reference to the push controller
     * @param wavefront_planner Reference to the wavefront planner
     */
    NAMOPlanner(NAMOEnvironment& env, 
               PushController& controller, 
               WavefrontPlanner& wavefront_planner)
        : env(env), 
          controller(controller), 
          wavefront_planner(wavefront_planner) {

        // Pre-allocate buffer capacities
        reachable_objects_buffer.reserve(20);
        robot_position_buffer.reserve(2);
        allowed_indices_buffer.reserve(100);
        // qpos_buffer.reserve(50);  // Adjust based on your state space dimension
        action_indices_buffer.reserve(100);
        cache_key_buffer.reserve(100);
        // transformed_edge_points_buffer will be populated as needed
        // No need to reserve for idx_set_buffer as sets grow dynamically
        path_buffer.reserve(200);
    }

    /**
     * @brief Computes wavefront from current robot position and optionally saves it
     * 
     * @param transformed_edges Edge points to consider in computation
     * @param output_path Optional path to save wavefront data
     * @return Wavefront computation results
     */
    auto computeAndSaveWavefront(
        const std::unordered_map<std::string, std::vector<std::array<double, 2>>>& transformed_edges,
        const std::string& output_path = "") {
        
        // Get current robot position
        auto robot_state = env.get_robot_state();
        robot_position_buffer.clear();
        robot_position_buffer.push_back(robot_state->position[0]);
        robot_position_buffer.push_back(robot_state->position[1]);
        
        // Compute wavefront
        auto wavefront_result = wavefront_planner.compute_wavefront(env, robot_position_buffer, transformed_edges);
        
        // Save to file if path is provided
        if (!output_path.empty()) {
            wavefront_planner.save_wavefront_to_file(output_path);
            env.increment_wavefront_id();
        }
        
        return wavefront_result;
    }
    
    /**
     * @brief Executes a primitive action if the edge is reachable
     * 
     * @param object_name Object to manipulate
     * @param edge_idx Edge index to push from
     * @param push_steps Number of push steps
     * @return bool Whether action was executed
     */
    bool executeActionIfReachable(
        const std::string& object_name,
        int edge_idx,
        int push_steps) {
        
        auto robot_state = env.get_robot_state();
        robot_position_buffer.clear();
        robot_position_buffer.push_back(robot_state->position[0]);
        robot_position_buffer.push_back(robot_state->position[1]);

        auto [all_edge_points, all_mid_points] = controller.get_object_edge_points_all();
        auto edge_points = all_edge_points[object_name];
        auto obj_info = env.get_object_info(object_name);
        
        transformed_edge_points_buffer.clear();
        transformed_edge_points_buffer[object_name] = MotionPrimitiveGenerator::transform_points(
            edge_points, obj_info->position, obj_info->quaternion);

        auto [wavefront, reachable_points, reachability_flags] = 
            wavefront_planner.compute_wavefront(env, robot_position_buffer, transformed_edge_points_buffer);

        if (reachability_flags[object_name][edge_idx] == 1) {
            qpos_buffer.clear();
            env.get_state_space()->copy_vector_from_point(qpos_buffer, env.get_qpos());
            controller.execute_primitive(qpos_buffer, object_name, push_steps, edge_idx);
            return true;
        }
        return false;
    }

    /**
     * @brief Creates a wavefront file path in the given directory
     * 
     * @param dir Directory to create path in
     * @param iter Iteration number for filename
     * @return Full file path
     */
    std::string createWavefrontFilePath(
        const std::filesystem::path& dir,
        int iter) {
        
        path_buffer.clear();
        path_buffer = "wavefront_data_" + std::to_string(iter) + ".txt";
        return (dir / path_buffer).string();
    }

    /**
     * @brief Creates a new directory for outputs and returns its path
     * 
     * @param parent_dir Parent directory
     * @param use_existing If true, will use existing folders, otherwise creates new one
     * @param existing_folder Optional specific folder number to use
     * @return New directory path
     */
    std::filesystem::path createOutputDirectory(
        const std::filesystem::path& parent_dir,
        bool use_existing = false,
        int existing_folder = -1) {
        
        // Count existing folders
        int folder_count = 0;
        for (const auto& entry : std::filesystem::directory_iterator(parent_dir)) {
            if (entry.is_directory()) {
                folder_count++;
            }
        }
        
        int folder_number = use_existing ? 
            (existing_folder >= 0 ? existing_folder : folder_count) : 
            (folder_count + 1);
            
        std::filesystem::path output_dir = parent_dir / std::to_string(folder_number);
        
        // Create directory if it doesn't exist
        if (!std::filesystem::exists(output_dir)) {
            std::filesystem::create_directory(output_dir);
        }
        
        return output_dir;
    }

    /**
     * @brief Performs the main planning loop to reach a global goal
     * 
     * @param robot_global_goal Target position for the robot
     * @param total_iter Maximum number of iterations
     * @param wavefronts_dir Directory to save wavefront data
     * @param action_steps Vector to store all action steps
     * @return bool True if goal was reached
     */
    bool performPlanningLoop(
        const std::vector<double>& robot_global_goal,
        int total_iter,
        const std::filesystem::path& wavefronts_dir,
        std::vector<ActionStep*>& action_steps) {
        
        int current_iter = 0;
        bool global_goal_reachable = false;
        
        // Create wavefront run directory - USING OUR NEW HELPER
        std::filesystem::path wavefront_run_dir = createOutputDirectory(wavefronts_dir);
        
        // Get edge points for all objects
        auto [all_edge_points, all_mid_points] = controller.get_object_edge_points_all();
        std::unordered_map<std::string, std::vector<std::array<double, 2>>> transformed_edge_points;
        
        // Enable logging and reset environment
        env.enable_logging();
        env.reset();
        
        // Main planning loop
        while(current_iter < total_iter) {
            // Update transformed edge points for current object states
            updateTransformedEdgePoints(all_edge_points, transformed_edge_points);

            // Compute wavefront and save to file
            std::string output_path = createWavefrontFilePath(wavefront_run_dir, current_iter);
            auto [wavefront, reachable_points, reachability_flags] = 
                computeAndSaveWavefront(transformed_edge_points, output_path);

            // Check if goal is reachable
            global_goal_reachable = wavefront_planner.is_goal_reachable(robot_global_goal, 0.3);

            if (global_goal_reachable) {
                for(int i = 0; i < 200; i++) {
                    env.update_object_states();
                }
                std::cout << "Goal reached, num action steps: " << action_steps.size() << std::endl;
                break;
            }

            // Get reachable objects
            std::vector<std::string>& reachable_objects = getReachableObjects(reachable_points);
            if (reachable_objects.empty()) {
                std::cout << "No reachable objects" << std::endl;
                return false;
            }

            // Select random object
            std::string random_object = selectRandomObject(reachable_objects);
            
            // Use our optimized function for allowed indices
            std::vector<int>& allowed_indices = getAllowedPrimitiveIndices(reachability_flags, random_object);
            
            // Set goal for selected object
            auto random_object_info = env.get_object_info(random_object);
            std::vector<double> goal_state = env.set_goal_configuration(random_object, 0.3, 0.6);
            
            // Execute push action
            auto [action_step, controller_success] = controller.execute_push_action(
                random_object, allowed_indices, goal_state);

            if (action_step != nullptr && action_step->push_steps > 0) {
                action_steps.push_back(action_step);
            }

            current_iter++;
        }
        
        return global_goal_reachable;
    }

    /**
     * @brief Optimizes the action sequence to find minimal steps
     * 
     * @param action_steps Vector of all action steps
     * @param robot_global_goal Target position for the robot
     * @param final_wavefronts_dir Directory to save final solution wavefronts
     * @return std::vector<std::vector<int>> Optimized action sequences
     */
    std::vector<std::vector<int>> optimizeActionSequence(
        const std::vector<ActionStep*>& action_steps,
        const std::vector<double>& robot_global_goal,
        const std::filesystem::path& final_wavefronts_dir) {
        
        env.disable_logging();
        
        std::unordered_map<int, std::vector<std::unordered_set<int>>> len_idx_set_map;
        std::unordered_set<std::string> unique_action_seq;
        std::unordered_map<std::string, bool> goal_reachability_cache;
        unique_action_seq.reserve(100);         // Reserve space for unique sequences
        
        env.reset();
        int action_steps_size = action_steps.size();
        std::stack<std::tuple<std::unordered_set<int>, ActionStep*, std::vector<double>>> action_stack;
        
        // Add action steps to stack in reverse order
        for(int i = action_steps_size - 1; i >= 0; i--) {
            // Use our buffer for qpos
            qpos_buffer.clear();
            env.get_state_space()->copy_vector_from_point(qpos_buffer, env.get_qpos());
            
            idx_set_buffer.clear();  // Clear our index set buffer
            idx_set_buffer.insert(i);
            action_stack.push(std::make_tuple(idx_set_buffer, action_steps[i], qpos_buffer));
        }

        int max_set_size = 0;
        int total_solns_max_set_size = 0;
        int ps_ctr = 0;
        int total_ps_ctr = 100000;

        auto start_time = std::chrono::high_resolution_clock::now();
        while(!action_stack.empty()) {
            if (total_solns_max_set_size > 20) {
                break;
            }
            ps_ctr++;
            if (ps_ctr % 5000 == 0) {
                std::cout << "ps_ctr: " << ps_ctr << " total_solns_max_set_size: " << total_solns_max_set_size << std::endl;
            }
            if (ps_ctr > total_ps_ctr) {
                break;
            }

            auto [idx_set, action_step, qpos] = action_stack.top();
            
            env.set_qpos(qpos);
            action_stack.pop();
            
            // Generate cache key from action indices - use our buffer
            cache_key_buffer.clear();  // Clear without deallocating
            for(int i = 0; i < action_steps_size; i++){
                if (idx_set.count(i) > 0) {
                    continue;
                }
                if (i < 26) {
                    cache_key_buffer += static_cast<char>('a' + i); 
                }
                else if (i < 52) {
                    cache_key_buffer += static_cast<char>('A' + (i - 26));
                }
                else{
                    cache_key_buffer += static_cast<char>('0' + (i - 52));
                }
            }

            // Check if goal is reachable with this action set
            bool goal_reachable = checkGoalReachable(
                cache_key_buffer, goal_reachability_cache, idx_set, max_set_size, 
                action_steps, action_steps_size, robot_global_goal);

            if (goal_reachable) {
                env.reset();
                if (len_idx_set_map.count(idx_set.size()) == 0) {
                    std::vector<std::unordered_set<int>> idx_set_vec;
                    if (idx_set.size() > max_set_size) {
                        max_set_size = idx_set.size();
                        total_solns_max_set_size = 0;
                    }
                    
                    len_idx_set_map[idx_set.size()] = idx_set_vec;
                }

                if (idx_set.size() == max_set_size) {
                    len_idx_set_map[idx_set.size()].push_back(idx_set);
                    if (unique_action_seq.count(cache_key_buffer) == 0) {
                        unique_action_seq.insert(cache_key_buffer);
                        total_solns_max_set_size += 1;
                    }
                }

                // Expand search by removing one more action
                for(int i = action_steps_size - 1; i >= 0; i--) {
                    idx_set_buffer = idx_set;  // Copy to our buffer
                    if (idx_set_buffer.count(i) > 0) {
                        continue;
                    }
                    
                    // Use our buffer for qpos
                    qpos_buffer.clear();
                    env.get_state_space()->copy_vector_from_point(qpos_buffer, env.get_qpos());

                    idx_set_buffer.insert(i);
                    action_stack.push(std::make_tuple(idx_set_buffer, action_steps[i], qpos_buffer));
                }

                goal_reachability_cache[cache_key_buffer] = true;
            }
            else{
                goal_reachability_cache[cache_key_buffer] = false;
            }
        }
        
        auto end_time = std::chrono::high_resolution_clock::now(); 
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        std::cout << "Optimization time: " << duration.count() << " milliseconds" << std::endl;

        // Convert and visualize action sequences
        std::vector<std::vector<int>> action_sequences = convertToActionSequences(
            unique_action_seq, action_steps, action_steps_size, max_set_size);

        visualizeActionSequences(action_sequences, action_steps, final_wavefronts_dir);
        
        return action_sequences;
    }

private:
    NAMOEnvironment& env;
    PushController& controller;
    WavefrontPlanner& wavefront_planner;
    
    // Add a reusable buffer as a member variable
    std::vector<std::string> reachable_objects_buffer;
    
    // Add these new buffers
    std::vector<double> robot_position_buffer;
    std::vector<int> allowed_indices_buffer;
    std::vector<double> qpos_buffer;
    std::vector<int> action_indices_buffer;
    std::string cache_key_buffer;
    std::unordered_map<std::string, std::vector<std::array<double, 2>>> transformed_edge_points_buffer;
    std::unordered_set<int> idx_set_buffer;
    std::string path_buffer;

    /**
     * @brief Checks if goal is reachable with a given set of actions
     */
    bool checkGoalReachable(
        const std::string& cache_key,
        std::unordered_map<std::string, bool>& goal_reachability_cache,
        const std::unordered_set<int>& idx_set,
        int max_set_size,
        const std::vector<ActionStep*>& action_steps,
        int action_steps_size,
        const std::vector<double>& robot_global_goal) {
        
        // Check if we already know the result
        if (goal_reachability_cache.count(cache_key) > 0) {
            return goal_reachability_cache[cache_key];
        }
        
        // If set is smaller than max set size, assume reachable (we don't care about these)
        if (idx_set.size() < max_set_size) {
            return true;
        }
        
        // Otherwise, simulate actions and check reachability
        for(int i = 0; i < action_steps_size; i++) {
            if (idx_set.count(i) > 0) {
                continue;
            }
            
            // USING OUR NEW HELPER - execute action if reachable
            ActionStep* action_step = action_steps[i];
            executeActionIfReachable(action_step->object_name, 
                                   action_step->edge_idx,
                                   action_step->push_steps);
        }

        // USING OUR NEW HELPER - compute final wavefront to check goal reachability
        auto [wavefront, reachable_points, reachability_flags] = computeAndSaveWavefront({});
        return wavefront_planner.is_goal_reachable(robot_global_goal, 0.3);
    }

    /**
     * @brief Converts unique action sequences to vector format
     */
    std::vector<std::vector<int>> convertToActionSequences(
        const std::unordered_set<std::string>& unique_action_seq,
        const std::vector<ActionStep*>& action_steps,
        int action_steps_size,
        int max_set_size) {
        
        std::vector<std::vector<int>> action_sequences;
        for (const auto& cache_key : unique_action_seq) {
            if (cache_key.size() != (action_steps_size - max_set_size)) {
                continue;
            }
            
            // Reuse our buffer for action indices
            action_indices_buffer.clear();
            for (int i = 0; i < cache_key.size(); i++) {
                action_indices_buffer.push_back(convert_ch_to_int(cache_key[i]));
            }

            std::cout << "Action sequence: \n";
            for (int idx : action_indices_buffer) {
                std::cout << action_steps[idx]->object_name 
                          << " steps:" << action_steps[idx]->push_steps 
                          << " edge:" << action_steps[idx]->edge_idx << std::endl;
            }   
            
            // We still need to push the resulting sequence into our return vector
            action_sequences.push_back(action_indices_buffer);
        }
        
        return action_sequences;
    }

    /**
     * @brief Visualizes and tests action sequences
     */
    void visualizeActionSequences(
        const std::vector<std::vector<int>>& action_sequences,
        const std::vector<ActionStep*>& action_steps,
        const std::filesystem::path& final_wavefronts_dir) {
        
        env.enable_logging();
        env.reset();

        // Start with the first folder
        int final_folder_count = 0;
        
        for(const auto& action_steps_indices: action_sequences) {
            // Create output directory - USING OUR NEW HELPER
            std::filesystem::path wavefront_run_dir = createOutputDirectory(
                final_wavefronts_dir, true, final_folder_count);
                
            int ctr = 0;

            for(int action_step_idx: action_steps_indices) {
                ActionStep* action_step = action_steps[action_step_idx];
                
                // Execute action if reachable
                executeActionIfReachable(action_step->object_name, 
                                      action_step->edge_idx,
                                      action_step->push_steps);
                
                // Save wavefront after action
                std::string output_path = createWavefrontFilePath(wavefront_run_dir, ctr);
                computeAndSaveWavefront({}, output_path);
                ctr++;
            }
            
            // Final wavefront after all actions
            std::string output_path = createWavefrontFilePath(wavefront_run_dir, ctr);
            computeAndSaveWavefront({}, output_path);

            for(int i = 0; i < 200; i++) {
                env.update_object_states();
            }

            final_folder_count++;
            env.reset();
        }
    }

    /**
     * @brief Updates transformed edge points for all objects
     */
    void updateTransformedEdgePoints(
        const std::unordered_map<std::string, std::vector<std::array<double, 2>>>& all_edge_points,
        std::unordered_map<std::string, std::vector<std::array<double, 2>>>& transformed_edge_points) {
        
        transformed_edge_points.clear();  // Clear instead of creating new each time
        
        for (const auto& [obj_name, edge_points] : all_edge_points) {
            auto obj_state = env.get_object_state(obj_name);
            
            // Make sure destination vector exists and has right size
            auto& dest_points = transformed_edge_points[obj_name];
            dest_points.resize(edge_points.size());
            
            // Transform points directly into our buffer
            // Note: We'd need to modify transform_points to accept a destination buffer
            // For now, we'll still use the original function
            dest_points = MotionPrimitiveGenerator::transform_points(
                edge_points, obj_state->position, obj_state->quaternion);
        }
    }

    /**
     * @brief Get list of reachable objects from planner results
     */
    std::vector<std::string>& getReachableObjects(
        const std::unordered_map<std::string, std::vector<std::array<double, 2>>>& reachable_points) {
        
        reachable_objects_buffer.clear();
        for (const auto& [obj_name, points] : reachable_points) {
            if (!points.empty()) {
                reachable_objects_buffer.push_back(obj_name);
            }
        }
        return reachable_objects_buffer;
    }

    /**
     * @brief Select a random object from the list of reachable objects
     */
    std::string selectRandomObject(const std::vector<std::string>& reachable_objects) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> idx_dis(0, reachable_objects.size() - 1);
        int random_index = idx_dis(gen);
        return reachable_objects[random_index];
    }

    /**
     * @brief Get allowed primitive indices for an object
     */
    std::vector<int>& getAllowedPrimitiveIndices(
        const std::unordered_map<std::string, std::vector<int>>& reachability_flags,
        const std::string& object_name) {
        
        allowed_indices_buffer.clear();
        for (size_t i = 0; i < reachability_flags.at(object_name).size(); i++) {
            if (reachability_flags.at(object_name)[i] == 1) {
                allowed_indices_buffer.push_back(i);
            }
        }
        return allowed_indices_buffer;
    }
};

} // namespace prx 