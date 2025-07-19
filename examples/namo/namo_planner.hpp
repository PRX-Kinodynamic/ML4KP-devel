#pragma once

#include "environment.hpp"
#include "push_controller.hpp"
#include "wavefront_planner.hpp"
#include "prx/utilities/communication/zmq_communication.hpp"
#include <unordered_map>
#include <unordered_set>
#include <nlohmann/json.hpp>
#include <fstream>  // For file operations
#include <iomanip>  // For std::setw

namespace prx {

// Define json type alias
using json = nlohmann::json;

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
               WavefrontPlanner& wavefront_planner,
               int object_strategy,
               bool diffusion_enabled, 
               bool diffusion_goal_enabled,
               const std::string endpoint)
        : env(env), 
          controller(controller), 
          wavefront_planner(wavefront_planner),
          object_strategy(object_strategy),
          diffusion_enabled(diffusion_enabled),
          diffusion_goal_enabled(diffusion_goal_enabled) {

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


        // // Initialize as REQ socket (client)
        // std::string endpoint = "tcp://arrakis.cs.rutgers.edu:5555";
        if (!client.initialize_socket(zmq_communication_t::socket_type::REQ, endpoint, false)) {
            std::cerr << "Failed to initialize client socket" << std::endl;
            communication_enabled = false;
        }
        else{
            communication_enabled = true;   
        }
    }

    /**
     * @brief Computes wavefront from current robot position and optionally saves it
     * 
     * @param transformed_edges Edge points to consider in computation
     * @param output_path Optional path to save wavefront data
     * @return Wavefront computation results
     */
    auto computeWavefront(
        const std::unordered_map<std::string, std::vector<std::array<double, 2>>>& transformed_edges,
        bool save_wavefront = false,
        const std::string& output_path = "") {
        
        // Get current robot position
        auto robot_state = env.get_robot_state();
        robot_position_buffer.clear();
        robot_position_buffer.push_back(robot_state->position[0]);
        robot_position_buffer.push_back(robot_state->position[1]);
        
        // Compute wavefront
        auto wavefront_result = wavefront_planner.compute_wavefront(env, robot_position_buffer, transformed_edges);
        
        // Save to file if path is provided
        if (save_wavefront) {
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
        ActionStepMPC* action_step,
        const std::array<double, 2>& robot_global_goal,
        bool save_wavefront = false, 
        const std::string& output_path = "",
        bool use_final_state = false) {

        const std::string& object_name = action_step->object_name;
        const std::vector<double>& goal_state = action_step->goal_state;
        const std::vector<double>& final_state = action_step->final_state;


        auto [all_edge_points, all_mid_points] = controller.get_object_edge_points_all();
        std::unordered_map<std::string, std::vector<std::array<double, 2>>> transformed_edge_points;
        updateTransformedEdgePoints(all_edge_points, transformed_edge_points);

        // check if object is reachable
        auto [wavefront, reachable_points, reachability_flags] = computeWavefront(transformed_edge_points, save_wavefront, output_path);
        getReachableObjects(reachable_points, true);

        if (reachable_objects_buffer.find(object_name) == reachable_objects_buffer.end()){
            // std::cout << "object is not reachable" << std::endl;
            return false;
        }

        if (use_final_state){
            executeMPC(object_name, final_state, 10, robot_global_goal);
        }
        else{
            executeMPC(object_name, goal_state, 10, robot_global_goal);
        }

        return true;
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


    
    bool executeMPC(
        const std::string& object_name,
        const std::vector<double>& goal_state,
        int mpc_steps,
        const std::array<double, 2>& robot_global_goal) {

        MujocoGoal goal;
        goal.position = {goal_state[0], goal_state[1], goal_state[2]};
        goal.orientation = {goal_state[3], goal_state[4], goal_state[5], goal_state[6]};
        goal.size = env.get_object_info(object_name)->size;  
        goal.geom_type = env.get_object_info(object_name)->geom_type;
        env.set_goal(goal);

        // returns true if global goal is reachable in the middle of the mpc
        // returns false if global goal is not reachable in the middle of the mpc

        // define a distance function for 2d and quaternion
        auto distance = [](const std::array<double, 3>& a, const std::array<double, 3>& b) {
            return std::sqrt(std::pow(a[0] - b[0], 2) + std::pow(a[1] - b[1], 2));
        };
        auto distance_quat = [](const std::array<double, 4>& a, const std::array<double, 4>& b) {
            return std::sqrt(std::pow(a[0] - b[0], 2) + std::pow(a[1] - b[1], 2) + std::pow(a[2] - b[2], 2) + std::pow(a[3] - b[3], 2));
        };

        double distance_threshold = 0.005;
        int same_state_ctr = 0;
        std::array<double, 3> prev_state = env.get_object_state(object_name)->position;
        std::array<double, 4> prev_quat = env.get_object_state(object_name)->quaternion;
        auto [all_edge_points, all_mid_points] = controller.get_object_edge_points_all();
        std::unordered_map<std::string, std::vector<std::array<double, 2>>> transformed_edge_points;

        for (int i = 0; i < mpc_steps; i++){

            updateTransformedEdgePoints(all_edge_points, transformed_edge_points);
            auto [wavefront_mpc, reachable_points_mpc, reachability_flags_mpc] = computeWavefront(transformed_edge_points, false, "");
            allowed_indices_buffer.clear();

            if (wavefront_planner.is_goal_reachable(robot_global_goal, 0.3)){
                // std::cout << "goal reached" << std::endl;
                return false;
            }

            getAllowedPrimitiveIndices(reachability_flags_mpc, object_name);

            if (allowed_indices_buffer.empty()){
                // std::cout << "no allowed indices" << std::endl;
                return false;
            }

            auto [action_step_ptr, controller_success] = controller.execute_push_action(object_name, allowed_indices_buffer, goal_state);

            if (controller_success){
                // std::cout << "reached goal" << std::endl;
                return false;
            }
            auto obj_state = env.get_object_state(object_name);
            if (distance(obj_state->position, prev_state) < distance_threshold && distance_quat(obj_state->quaternion, prev_quat) < distance_threshold){
                same_state_ctr++;
            }
            else{
                same_state_ctr = 0;
            }
            if (same_state_ctr > 1){
                    // std::cout << "same state for 2 times, returning" << std::endl;
                    return false;
            }
            prev_state = obj_state->position;
            prev_quat = obj_state->quaternion;
        }
        return false;
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
        const std::array<double, 2>& robot_global_goal,
        int total_iter,
        const std::filesystem::path& wavefronts_dir,
        std::vector<std::unique_ptr<ActionStepMPC>>& action_steps) {
        
        int current_iter = 0;
        bool global_goal_reachable = false;
        
        // Create wavefront run directory - USING OUR NEW HELPER
        std::filesystem::path wavefront_run_dir = createOutputDirectory(wavefronts_dir);
        
        // Get edge points for all objects
        auto [all_edge_points, all_mid_points] = controller.get_object_edge_points_all();
        std::unordered_map<std::string, std::vector<std::array<double, 2>>> transformed_edge_points;
        
        // Enable logging and reset environment
        // env.enable_logging();
        env.reset();


        bool fresh_start = true;
        
        // Main planning loop
        while(current_iter < total_iter) {
            // Update transformed edge points for current object states
            updateTransformedEdgePoints(all_edge_points, transformed_edge_points);
            // Compute wavefront and save to file
            std::string output_path = createWavefrontFilePath(wavefront_run_dir, current_iter);
            auto [wavefront, reachable_points, reachability_flags] = computeWavefront(transformed_edge_points, false, "");
                // computeWavefront(transformed_edge_points, true, output_path);
                
            // wavefront_planner.save_wavefront_to_file("wavefront_test.txt");

            // Check if goal is reachable
            // std::cout << "robot_global_goal: " << robot_global_goal[0] << " " << robot_global_goal[1] << std::endl;
            global_goal_reachable = wavefront_planner.is_goal_reachable(robot_global_goal, 0.3);

            if (global_goal_reachable) {
                for(int i = 0; i < 200; i++) {
                    env.update_object_states();
                }
                // std::cout << "Goal reached, num action steps: " << action_steps.size() << std::endl;
                break;
            }

            getReachableObjects(reachable_points, true);
            for (const auto& obj : reachable_objects_buffer){
                // std::cout << "reachable_object: " << obj << std::endl;
            }

            if (reachable_objects_buffer.empty()){
                // std::cout << "No reachable objects" << std::endl;
                current_iter++;
                return false;
            }

            std::string random_object = "";
            std::vector<double> goal_state;
        
            if (object_strategy == 0) {
                
                //select object
                random_object = selectRandomObject(reachable_objects_buffer);
                reachable_objects_buffer.erase(random_object);
                std::cout << "random_object: " << random_object << std::endl;

                int while_ctr = 0;
                int max_while_ctr = 100;
                while(while_ctr < max_while_ctr){
                    allowed_indices_buffer.clear();

                    getAllowedPrimitiveIndices(reachability_flags, random_object);

                    if (allowed_indices_buffer.empty()){
                        // std::cout << "no allowed indices during object selection" << std::endl;
                        if (reachable_objects_buffer.empty()){
                            current_iter++;
                            return false;
                        }
                        random_object = selectRandomObject(reachable_objects_buffer);
                        // std::cout << "picking new random object: " << random_object << std::endl;
                        reachable_objects_buffer.erase(random_object);
                    }
                    else{
                        break;
                    }
                    while_ctr++;
                };
                

                // Set goal for selected object
                // auto random_object_info = env.get_object_info(random_object);
                int mpc_steps = 10;
                int mpc_ctr = 0;
                std::vector<double> goal_state = set_goal_configuration(random_object, 0.3, 1.0);
                global_goal_reachable = executeMPC(random_object, goal_state, mpc_steps, robot_global_goal);

                auto obj_state = env.get_object_state(random_object);
                std::unique_ptr<ActionStepMPC> action_step_mpc = nullptr;
                action_step_mpc = std::make_unique<ActionStepMPC>();
                action_step_mpc->object_name = random_object;

                // if global goal is reachable, then use the final state as the goal state
                // otherwise, use the goal state as the goal state
                if (global_goal_reachable){
                    action_step_mpc->goal_state = {obj_state->position[0], obj_state->position[1], 0.0, obj_state->quaternion[0], obj_state->quaternion[1], obj_state->quaternion[2], obj_state->quaternion[3]};
                }
                else{
                    action_step_mpc->goal_state = goal_state;
                }
                
                action_step_mpc->final_state = {obj_state->position[0], obj_state->position[1], 0.0, obj_state->quaternion[0], obj_state->quaternion[1], obj_state->quaternion[2], obj_state->quaternion[3]};


                action_steps.push_back(std::move(action_step_mpc));

                // while (mpc_ctr < mpc_steps){
                //     updateTransformedEdgePoints(all_edge_points, transformed_edge_points);
                //     auto [wavefront_mpc, reachable_points_mpc, reachability_flags_mpc] = computeWavefront(transformed_edge_points, false, "");
                //     allowed_indices_buffer.clear();

                //     global_goal_reachable = wavefront_planner.is_goal_reachable(robot_global_goal, 0.3);

                //     if (global_goal_reachable){
                //         std::cout << "goal reached" << std::endl;
                //         break;
                //     }

                //     getAllowedPrimitiveIndices(reachability_flags_mpc, random_object);

                //     if (allowed_indices_buffer.empty()){
                //         std::cout << "no allowed indices" << std::endl;
                //         mpc_ctr++;
                //         continue;
                //     }
                //     auto [action_step_ptr, controller_success] = controller.execute_push_action(random_object, allowed_indices_buffer, goal_state);

                //     if (controller_success){
                //         std::cout << "reached goal" << std::endl;
                //         break;
                //     }
                //     mpc_ctr++;
                // }


                // while(goal_iter < max_goal_iter){
                
                //     goal_state = set_goal_configuration(random_object, 0.3, 1.0);

                //     // wait for user input
                //     // Execute push action
                    
                //     auto [action_step_ptr, controller_success] = controller.execute_push_action(
                //         random_object, allowed_indices_buffer, goal_state);


                //     if (action_step_ptr && action_step_ptr->push_steps > 0) {
                //         action_steps.push_back(std::move(action_step_ptr));
                //         break;
                //     }
                //     goal_iter++;
                // }
            }

            else if (object_strategy == 1) {
                // send zmq request to send json of current state and receive a json of object name and goal state
                json state_json = createStateJson();
                state_json["msg_type"] = "decision_req";
                state_json["robot_goal"] = robot_global_goal;
                state_json["reachable_objects"] = json::array();
                for (const auto& obj : reachable_objects_buffer){
                    state_json["reachable_objects"].push_back(obj);
                }
                std::string state_json_str = state_json.dump();

                if (client.send_message(state_json_str)) {
                    std::string response = client.receive_message();
                    std::cout << "response: " << response << std::endl;
                    
                    try {
                        json response_json = json::parse(response);
                        
                        // Check for error in response
                        if (response_json.contains("error") && response_json["error"]) {
                            global_goal_reachable = false;
                            current_iter++;
                            break;
                        }
                        
                        // Validate required fields
                        if (!response_json.contains("object") || !response_json.contains("goal_center") || !response_json.contains("final_quat")) {
                            std::cerr << "Invalid response: missing required fields" << std::endl;
                            current_iter++;
                            continue;
                        }
                        
                        random_object = response_json["object"];
                        std::vector<double> goal_center = response_json["goal_center"];
                        std::vector<double> goal_quat = response_json["final_quat"];

                        // Validate array sizes
                        if (goal_center.size() < 2 || goal_quat.size() < 4) {
                            std::cerr << "Invalid response: insufficient array sizes" << std::endl;
                            current_iter++;
                            continue;
                        }

                        // Fix: Create proper 7-element goal state with z=0.0
                        goal_state = {goal_center[0], goal_center[1], 0.0, goal_quat[0], goal_quat[1], goal_quat[2], goal_quat[3]};

                        auto object_info = env.get_object_info(random_object);
                        if (!object_info) {
                            std::cerr << "Invalid object name received: " << random_object << std::endl;
                            current_iter++;
                            continue;
                        }

                        MujocoGoal goal;
                        // Fix: Set proper 3D position
                        goal.position = {goal_state[0], goal_state[1], goal_state[2]};
                        // Fix: Set proper quaternion indices (3,4,5,6)
                        goal.orientation = {goal_state[3], goal_state[4], goal_state[5], goal_state[6]};
                        goal.size = object_info->size;  
                        goal.geom_type = object_info->geom_type;
                        env.set_goal(goal);

                        // allow for user to press enter to continue
                        // std::cout << "Press enter to continue" << std::endl;
                        // std::cin.ignore();

                        allowed_indices_buffer.clear();
                        getAllowedPrimitiveIndices(reachability_flags, random_object);

                        // Fix: Check for allowed indices BEFORE trying to execute action
                        if (allowed_indices_buffer.empty()){
                            std::cout << "No allowed indices for object: " << random_object << std::endl;
                            current_iter++;
                            continue;
                        }

                        // Execute MPC instead of single push action to match object_strategy == 0
                        global_goal_reachable = executeMPC(random_object, goal_state, 10, robot_global_goal);

                        auto obj_state = env.get_object_state(random_object);
                        std::unique_ptr<ActionStepMPC> action_step_mpc = std::make_unique<ActionStepMPC>();
                        action_step_mpc->object_name = random_object;

                        // Use the same logic as object_strategy == 0
                        if (global_goal_reachable){
                            action_step_mpc->goal_state = {obj_state->position[0], obj_state->position[1], 0.0, obj_state->quaternion[0], obj_state->quaternion[1], obj_state->quaternion[2], obj_state->quaternion[3]};
                        }
                        else{
                            action_step_mpc->goal_state = goal_state;
                        }
                        
                        action_step_mpc->final_state = {obj_state->position[0], obj_state->position[1], 0.0, obj_state->quaternion[0], obj_state->quaternion[1], obj_state->quaternion[2], obj_state->quaternion[3]};

                        action_steps.push_back(std::move(action_step_mpc));

                    } catch (const json::parse_error& e) {
                        std::cerr << "JSON parse error: " << e.what() << std::endl;
                        current_iter++;
                        continue;
                    }
                } else {
                    std::cerr << "Failed to send message" << std::endl;
                    current_iter++;
                    continue;
                }
            }
            
            
            current_iter++;

        }


        if (object_strategy == 1){
                // send zmq request to send json of current state and recieve a json of object name and goal state
                json state_json = createStateJson();
                state_json["msg_type"] = "result_info";
                state_json["success"] = false;
                std::string config_name = env.get_config_name();
                state_json["config_name"] = config_name;
                state_json["current_iter"] = current_iter;
                state_json["action_steps"] = action_steps.size();

                if (global_goal_reachable){
                    state_json["success"] = true;
                }

                std::string state_json_str = state_json.dump();
                if (client.send_message(state_json_str)) {
                    std::string response = client.receive_message();
                    // std::cout << "response: " << response << std::endl;
                }
                else {
                    std::cerr << "Failed to send message" << std::endl;
                }
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
        const std::vector<std::unique_ptr<ActionStepMPC>>& action_steps,
        const std::array<double, 2>& robot_global_goal,
        const std::filesystem::path& final_wavefronts_dir) {
        
        env.disable_logging();
        
        std::unordered_map<int, std::vector<std::unordered_set<int>>> len_idx_set_map;
        std::unordered_set<std::string> unique_action_seq;
        std::unordered_map<std::string, bool> goal_reachability_cache;
        unique_action_seq.reserve(100);         // Reserve space for unique sequences
        
        env.reset();
        int action_steps_size = action_steps.size();
        std::stack<std::tuple<std::unordered_set<int>, const ActionStepMPC*, std::vector<double>>> action_stack;
        
        // Add action steps to stack in reverse order
        for(int i = -1; i >= -1; i--) {
            // Use our buffer for qpos
            qpos_buffer.clear();
            env.get_state_space()->copy_vector_from_point(qpos_buffer, env.get_qpos());
            
            idx_set_buffer.clear();  // Clear our index set buffer
            // idx_set_buffer.insert(i);

            if (i < 0){
                action_stack.push(std::make_tuple(idx_set_buffer, nullptr, qpos_buffer));
            }
            else{
                action_stack.push(std::make_tuple(idx_set_buffer, action_steps[i].get(), qpos_buffer));
            }
        }

        int max_set_size = 0;
        int total_solns_max_set_size = 0;
        int ps_ctr = 0;
        int total_ps_ctr = 100000;

        auto start_time = std::chrono::high_resolution_clock::now();

        // std::cout << "action_stack.size(): " << action_stack.size() << std::endl;

        while(!action_stack.empty()) {
            if (total_solns_max_set_size > 20) {
                break;
            }
            ps_ctr++;
            if (ps_ctr % 1000 == 0) {
                // std::cout << "ps_ctr: " << ps_ctr << " total_solns_max_set_size: " << total_solns_max_set_size << std::endl;
            }
            if (ps_ctr > total_ps_ctr) {
                break;
            }

            auto [idx_set, action_step, qpos] = action_stack.top();

            action_stack.pop();
            
            if (idx_set.size() < max_set_size) {
                continue;
            }
            env.set_qpos(qpos);
            
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


            // std::cout << "cache_key_buffer: " << cache_key_buffer << std::endl;

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
                    action_stack.push(std::make_tuple(idx_set_buffer, action_steps[i].get(), qpos_buffer));
                }

                goal_reachability_cache[cache_key_buffer] = true;

                // std::cout << "cache_key_buffer: " << cache_key_buffer << " goal_reachable: " << goal_reachable << std::endl;
            }
            else{
                goal_reachability_cache[cache_key_buffer] = false;
            }
        }
        
        auto end_time = std::chrono::high_resolution_clock::now(); 
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        // std::cout << "Optimization time: " << duration.count() << " milliseconds" << std::endl;


        // Convert optimized action sequences
        std::vector<std::vector<int>> action_sequences = convertToActionSequences(
            unique_action_seq, action_steps, action_steps_size, max_set_size);
        
        // If no optimized sequences were found, use the original sequence
        // if (action_sequences.empty() && action_steps_size > 0) {
        //     std::cout << "No optimized sequences found. Using original action sequence." << std::endl;
            
        //     // Create original sequence with indices 0 to action_steps_size-1
        //     std::vector<int> original_sequence;
        //     original_sequence.reserve(action_steps_size);
        //     for (int i = 0; i < action_steps_size; i++) {
        //         original_sequence.push_back(i);
        //     }
            
        //     action_sequences.push_back(original_sequence);
            
        //     // Print the original sequence
        //     std::cout << "Original action sequence: \n";
        //     for (int idx : original_sequence) {
        //         std::cout << action_steps[idx]->object_name 
        //                   << " steps:" << action_steps[idx]->push_steps 
        //                   << " edge:" << action_steps[idx]->edge_idx << std::endl;
        //     }
        // }

        // visualizeActionSequences(action_sequences, robot_global_goal, action_steps, final_wavefronts_dir);
        return action_sequences;
    }

    /**
     * @brief Alternative optimization that starts with the last action and works backwards
     * 
     * @param action_steps Vector of all action steps
     * @param robot_global_goal Target position for the robot
     * @param final_wavefronts_dir Directory to save final solution wavefronts
     * @return std::vector<std::vector<int>> Optimized action sequences
     */
    std::vector<std::vector<int>> optimizeActionSequence2(
        const std::vector<std::unique_ptr<ActionStepMPC>>& action_steps,
        const std::array<double, 2>& robot_global_goal,
        const std::filesystem::path& final_wavefronts_dir) {
        
        env.disable_logging();
        
        int action_steps_size = action_steps.size();
        std::vector<std::vector<int>> working_sequences;
        
        // std::cout << "Starting optimization with " << action_steps_size << " actions" << std::endl;
        
        // Start with the last action and work backwards
        for (int seq_length = 1; seq_length <= action_steps_size; seq_length++) {
            // std::cout << "Testing sequences of length " << seq_length << " ending with last action" << std::endl;
            
            bool found_working_sequence = false;
            
            if (seq_length == 1) {
                // Test just the last action
                std::vector<int> sequence = {action_steps_size - 1};
                
                if (testActionSequence(sequence, action_steps, robot_global_goal)) {
                    working_sequences.push_back(sequence);
                    found_working_sequence = true;
                }
            } else {
                // Generate all combinations of (seq_length - 1) actions from the first (n-1) actions
                // Then append the last action
                std::vector<int> available_actions(action_steps_size - 1);
                std::iota(available_actions.begin(), available_actions.end(), 0); // 0, 1, 2, ..., n-2
                
                // Generate all combinations of (seq_length - 1) elements from available_actions
                std::vector<bool> selector(action_steps_size - 1);
                std::fill(selector.begin(), selector.begin() + (seq_length - 1), true);
                
                do {
                    // Create current sequence from selector + last action
                    std::vector<int> current_sequence;
                    for (int i = 0; i < action_steps_size - 1; i++) {
                        if (selector[i]) {
                            current_sequence.push_back(i);
                        }
                    }
                    // Always add the last action
                    current_sequence.push_back(action_steps_size - 1);
                    
                    // Test if this sequence works
                    // std::cout << "Testing sequence: ";
                    // for (int idx : current_sequence) {
                    //     std::cout << static_cast<char>('a' + idx) << " ";
                    // }
                    // std::cout << std::endl;
                    
                    if (testActionSequence(current_sequence, action_steps, robot_global_goal)) {
                        working_sequences.push_back(current_sequence);
                        found_working_sequence = true;
                    }
                    
                } while (std::prev_permutation(selector.begin(), selector.end()));
            }
            
            // If we found working sequences of this length, we're done
            if (found_working_sequence) {
                // std::cout << "Found " << working_sequences.size() 
                //           << " working sequences of length " << seq_length << std::endl;
                break;
            }
        }
        
        // if (working_sequences.empty()) {
        //     std::cout << "No working sequences found!" << std::endl;
        // }
        
        return working_sequences;
    }

    /**
     * @brief Helper function to create state JSON for robot and objects
     * 
     * @param env Reference to environment
     * @return json JSON object with robot and object states
     */
    json createStateJson() {
        json state_data;
        
        // Add robot state
        auto robot_state = env.get_robot_state();
        state_data["robot"] = {
            {"position", {robot_state->position[0], robot_state->position[1], robot_state->position[2]}},
            {"quaternion", {robot_state->quaternion[0], robot_state->quaternion[1], 
                        robot_state->quaternion[2], robot_state->quaternion[3]}}
        };
        
        // Add object states
        json objects_data = json::object();
        const auto& all_object_states = env.get_all_object_states();
        
        
        for (const auto& [obj_name, obj_state] : all_object_states) {
            if (obj_name != "robot") {
                objects_data[obj_name] = {
                    {"position", {obj_state.position[0], obj_state.position[1], obj_state.position[2]}},
                    {"quaternion", {obj_state.quaternion[0], obj_state.quaternion[1], 
                                obj_state.quaternion[2], obj_state.quaternion[3]}}
                };
            }
        }

        for (const auto& obj_state : env.get_static_objects()) {
            std::string obj_name = obj_state.name;
            objects_data[obj_name] = {
                {"position", {obj_state.position[0], obj_state.position[1], obj_state.position[2]}},
                {"quaternion", {obj_state.quaternion[0], obj_state.quaternion[1], 
                                obj_state.quaternion[2], obj_state.quaternion[3]}}
            };
        }


        state_data["objects"] = objects_data;

        std::string config_name = env.get_config_name();
        state_data["config_name"] = config_name;
        
        return state_data;
    }

    /**
     * @brief Collects and stores state-action pairs for all optimized sequences
     * 
     * @param action_sequences Vector of optimized action sequences
     * @param action_steps Original action steps
     * @param output_dir Directory to save the collected data
     * @param experiment_id Unique identifier for this experiment run
     * @return int Number of data points collected
     */
    int collectStateActionPairs(
        const std::vector<std::vector<int>>& action_sequences,
        const std::vector<std::unique_ptr<ActionStepMPC>>& action_steps,
        const std::filesystem::path& output_dir,
        const std::string& experiment_id,
        bool use_final_state = false) {
        
        // Make sure output directory exists
        if (!std::filesystem::exists(output_dir)) {
            std::filesystem::create_directory(output_dir);
        }
        
        int data_point_count = 0;

        // For each optimized sequence
        for (size_t seq_idx = 0; seq_idx < action_sequences.size(); seq_idx++) {
            
            // Reset environment to initial state
            env.reset();
            env.step_simulation();
            
            // Prepare sequence-specific output file
            std::string sequence_filename = "sequence_" + experiment_id + "_" + std::to_string(seq_idx) + ".json";
            std::filesystem::path sequence_file_path = output_dir / sequence_filename;
            
            // Create JSON structure for this sequence
            json sequence_data;
            sequence_data["experiment_id"] = experiment_id;
            sequence_data["sequence_id"] = seq_idx;
            sequence_data["config_name"] = env.get_config_name();
            std::array<double, 2> robot_goal = env.get_robot_goal();
            sequence_data["robot_goal"] = {robot_goal[0], robot_goal[1]};
            sequence_data["data_points"] = json::array();
            sequence_data["unoptimized_sequence_size"] = action_steps.size();
            sequence_data["optimized_sequence_size"] = action_sequences[seq_idx].size();

            // if (action_sequences[seq_idx].size() == 0) {
            //     continue;
            // }

            // Get edge points for all objects
            auto [all_edge_points, all_mid_points] = controller.get_object_edge_points_all();
            std::unordered_map<std::string, std::vector<std::array<double, 2>>> transformed_edge_points;
            
            // Execute each action in sequence and collect state-action pairs
            for (size_t step_idx = 0; step_idx < action_sequences[seq_idx].size(); step_idx++) {
                // Get current state before action
                auto robot_state = env.get_robot_state();

                updateTransformedEdgePoints(all_edge_points, transformed_edge_points);
                auto [wavefront, reachable_points, reachability_flags] = computeWavefront(transformed_edge_points, false, "");

                getReachableObjects(reachable_points, true);
                
                // Record object positions and orientations in state
                json state_data;
                state_data["robot"] = {
                    {"position", {robot_state->position[0], robot_state->position[1], robot_state->position[2]}},
                    {"quaternion", {robot_state->quaternion[0], robot_state->quaternion[1], 
                                   robot_state->quaternion[2], robot_state->quaternion[3]}}
                };

                state_data["reachable_objects"] = json::array();
                for (const auto& obj : reachable_objects_buffer){
                    state_data["reachable_objects"].push_back(obj);
                }
                
                // Get all object states and filter out non-movable ones
                json objects_data = json::object();
                const auto& all_object_states = env.get_all_object_states();
                
                for (const auto& [obj_name, obj_state] : all_object_states) {
                    if (obj_name != "robot") {
                        objects_data[obj_name] = {
                            {"position", {obj_state.position[0], obj_state.position[1], obj_state.position[2]}},
                            {"quaternion", {obj_state.quaternion[0], obj_state.quaternion[1], 
                                           obj_state.quaternion[2], obj_state.quaternion[3]}}
                        };
                    }
                }

                for (const auto& obj_state : env.get_static_objects()) {
                    std::string obj_name = obj_state.name;
                    objects_data[obj_name] = {
                        {"position", {obj_state.position[0], obj_state.position[1], obj_state.position[2]}},
                        {"quaternion", {obj_state.quaternion[0], obj_state.quaternion[1], 
                                        obj_state.quaternion[2], obj_state.quaternion[3]}}
                    };
                }
                state_data["objects"] = objects_data;
                
                // Get action
                int action_step_idx = action_sequences[seq_idx][step_idx];
                ActionStepMPC* action_step = action_steps[action_step_idx].get();

                if (reachable_objects_buffer.count(action_step->object_name) == 0){
                    continue;
                }

                // Create action data with only fields available in ActionStepMPC
                json action_data = {
                    {"object_name", action_step->object_name}
                };

                action_data["goal_state"] = {
                    {"position", {action_step->goal_state[0], action_step->goal_state[1], action_step->goal_state[2]}},
                    {"quaternion", {action_step->goal_state[3], action_step->goal_state[4], 
                                   action_step->goal_state[5], action_step->goal_state[6]}}
                };

                action_data["final_state"] = {
                    {"position", {action_step->final_state[0], action_step->final_state[1], action_step->final_state[2]}},
                    {"quaternion", {action_step->final_state[3], action_step->final_state[4], 
                                   action_step->final_state[5], action_step->final_state[6]}}
                };

                // Execute the action using executeMPC (which works with ActionStepMPC)
                // This will advance the environment state

                if (use_final_state){
                    executeMPC(action_step->object_name, action_step->final_state, 10, robot_goal);

                    action_data["which_state"] = "final";
                }
                else{
                    executeMPC(action_step->object_name, action_step->goal_state, 10, robot_goal);
                    action_data["which_state"] = "goal";
                }
                
                // Create the state-action pair data point
                json data_point = {
                    {"state", state_data},
                    {"action", action_data},
                    {"step_idx", step_idx},
                };
                sequence_data["data_points"].push_back(data_point);
                data_point_count++;
            }

            // Get final state after all actions
            auto robot_state = env.get_robot_state();

            updateTransformedEdgePoints(all_edge_points, transformed_edge_points);
            auto [wavefront, reachable_points, reachability_flags] = computeWavefront(transformed_edge_points, false, "");
            getReachableObjects(reachable_points, true);
            
            // Record final state
            json final_state_data;
            final_state_data["robot"] = {
                {"position", {robot_state->position[0], robot_state->position[1], robot_state->position[2]}},
                {"quaternion", {robot_state->quaternion[0], robot_state->quaternion[1], 
                                robot_state->quaternion[2], robot_state->quaternion[3]}}
            };
            
            final_state_data["reachable_objects"] = json::array();
            for (const auto& obj : reachable_objects_buffer){
                final_state_data["reachable_objects"].push_back(obj);
            }

            // Get all object states
            json objects_data = json::object();
            const auto& all_object_states = env.get_all_object_states();
            
            for (const auto& [obj_name, obj_state] : all_object_states) {
                if (obj_name != "robot") {
                    objects_data[obj_name] = {
                        {"position", {obj_state.position[0], obj_state.position[1], obj_state.position[2]}},
                        {"quaternion", {obj_state.quaternion[0], obj_state.quaternion[1], 
                                        obj_state.quaternion[2], obj_state.quaternion[3]}}
                    };
                }
            }
            
            for (const auto& obj_state : env.get_static_objects()) {
                std::string obj_name = obj_state.name;
                objects_data[obj_name] = {
                    {"position", {obj_state.position[0], obj_state.position[1], obj_state.position[2]}},
                    {"quaternion", {obj_state.quaternion[0], obj_state.quaternion[1], 
                                    obj_state.quaternion[2], obj_state.quaternion[3]}}
                };
            }
            final_state_data["objects"] = objects_data;

            // Add final state to sequence data
            sequence_data["data_points"].push_back({
                {"state", final_state_data},
            });

            // Save the sequence data to file
            std::ofstream sequence_file(sequence_file_path);
            sequence_file << std::setw(4) << sequence_data << std::endl;
            sequence_file.close();

            break;
        }
        return data_point_count;
    }

    std::vector<std::vector<int>> optimizeActionSequenceGreedy(
        const std::vector<std::unique_ptr<ActionStepMPC>>& action_steps,
        const std::array<double, 2>& robot_global_goal) {
        
        // Start with full sequence
        std::vector<int> current_sequence;
        for (int i = 0; i < action_steps.size(); i++) {
            current_sequence.push_back(i);
        }
        
        bool improved = true;
        while (improved) {
            improved = false;
            
            // Try removing each action (except last)
            for (int i = 0; i < current_sequence.size() - 1; i++) {
                std::vector<int> test_sequence = current_sequence;
                test_sequence.erase(test_sequence.begin() + i);
                
                if (testActionSequence(test_sequence, action_steps, robot_global_goal)) {
                    current_sequence = test_sequence;
                    improved = true;
                    break; // Found improvement, restart
                }
            }
        }
        
        return {current_sequence};
    }

private:
    NAMOEnvironment& env;
    PushController& controller;
    WavefrontPlanner& wavefront_planner;

    zmq_communication_t client;
    bool communication_enabled = false;
    bool diffusion_enabled;
    bool diffusion_goal_enabled;
    int object_strategy;
    
    // Add a reusable buffer as a member variable
    std::unordered_set<std::string> reachable_objects_buffer;
    
    // Add these new buffers
    std::vector<double> robot_position_buffer;
    std::vector<int> allowed_indices_buffer;
    std::vector<double> qpos_buffer;
    std::vector<int> action_indices_buffer;
    std::string cache_key_buffer;
    std::unordered_set<int> idx_set_buffer;
    std::string path_buffer;
    std::unordered_map<std::string, std::vector<std::array<double, 2>>> transformed_edge_points_buffer;
    std::unordered_map<std::string, std::vector<std::array<double, 2>>> transformed_mid_points_buffer;

    /**
     * @brief Checks if goal is reachable with a given set of actions
     */
    bool checkGoalReachable(
        const std::string& cache_key,
        std::unordered_map<std::string, bool>& goal_reachability_cache,
        const std::unordered_set<int>& idx_set,
        int max_set_size,
        const std::vector<std::unique_ptr<ActionStepMPC>>& action_steps,
        int action_steps_size,
        const std::array<double, 2>& robot_global_goal) {

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
            ActionStepMPC* action_step = action_steps[i].get();
            if (!executeActionIfReachable(action_step, robot_global_goal, false, "", false)) {
                return false;
            }
        }

        // USING OUR NEW HELPER - compute final wavefront to check goal reachability
        auto [wavefront, reachable_points, reachability_flags] = computeWavefront({}, false);
        return wavefront_planner.is_goal_reachable(robot_global_goal, 0.3);
    }

    /**
     * @brief Converts unique action sequences to vector format
     */
    std::vector<std::vector<int>> convertToActionSequences(
        const std::unordered_set<std::string>& unique_action_seq,
        const std::vector<std::unique_ptr<ActionStepMPC>>& action_steps,
        int action_steps_size,
        int max_set_size) {
        
        std::vector<std::vector<int>> action_sequences;
        for (const auto& cache_key : unique_action_seq) {
            // std::cout << "cache_key: " << cache_key << std::endl;
            // std::cout << "cache_key.size(): " << cache_key.size() << std::endl;
            // std::cout << "action_steps_size: " << action_steps_size << std::endl;
            // std::cout << "max_set_size: " << max_set_size << std::endl;
            // std::cout << "action_steps_size - max_set_size: " << action_steps_size - max_set_size << std::endl;

            if (cache_key.size() != (action_steps_size - max_set_size)) {
                continue;
            }
            
            // Reuse our buffer for action indices
            action_indices_buffer.clear();
            for (int i = 0; i < cache_key.size(); i++) {
                action_indices_buffer.push_back(convert_ch_to_int(cache_key[i]));
            }

            // std::cout << "Action sequence: \n";
            // for (int idx : action_indices_buffer) {
            //     std::cout << action_steps[idx]->object_name 
            //               << " steps:" << action_steps[idx]->push_steps 
            //               << " edge:" << action_steps[idx]->edge_idx << std::endl;
            // }   
            
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
        const std::array<double, 2>& robot_global_goal,
        const std::vector<std::unique_ptr<ActionStepMPC>>& action_steps,
        const std::filesystem::path& final_wavefronts_dir) {
        
        // env.enable_logging();
        env.reset();

        // Start with the first folder
        int final_folder_count = 0;
        
        for(const auto& action_steps_indices: action_sequences) {
            // Create output directory - USING OUR NEW HELPER
            std::filesystem::path wavefront_run_dir = createOutputDirectory(
                final_wavefronts_dir, true, final_folder_count);
                
            int ctr = 0;

            for(int action_step_idx: action_steps_indices) {
                ActionStepMPC* action_step = action_steps[action_step_idx].get();
                
                std::string output_path = createWavefrontFilePath(wavefront_run_dir, ctr);
                // Execute action if reachable
                executeActionIfReachable(action_step, robot_global_goal, true, output_path);

                
                // Save wavefront after action
                // computeWavefront({}, true, output_path);
                ctr++;
            }
            
            // Final wavefront after all actions
            std::string output_path = createWavefrontFilePath(wavefront_run_dir, ctr);
            computeWavefront({}, true, output_path);

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
   void getReachableObjects(
        const std::unordered_map<std::string, std::vector<std::array<double, 2>>>& reachable_points, bool use_reachable_points = true) {
        
        reachable_objects_buffer.clear();
        for (const auto& [obj_name, points] : reachable_points) {
            if (!points.empty() && use_reachable_points) {
                reachable_objects_buffer.insert(obj_name);
            }
            else {
                reachable_objects_buffer.insert(obj_name);
            }
        }
    }

    /**
     * @brief Select a random object from the list of reachable objects
     */
    std::string selectRandomObject(const std::unordered_set<std::string>& reachable_objects) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> idx_dis(0, reachable_objects.size() - 1);
        int random_index = idx_dis(gen);
        auto it = reachable_objects.begin();
        std::advance(it, random_index);
        return *it;
    }

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

    std::vector<double> set_goal_configuration(const std::string& object_name, 
                                             double min_distance = 0.3, 
                                             double max_distance = 0.6) {
        // Get object information from environment
        auto object_info = env.get_object_info(object_name);
        auto object_state = env.get_object_state(object_name);
        if (!object_info) {
            throw std::runtime_error("Object not found: " + object_name);
        }
        
        // Get environment bounds from environment
        std::vector<double> bounds = env.get_environment_bounds();
        double x_min = bounds[0], x_max = bounds[1];
        double y_min = bounds[2], y_max = bounds[3];
        
        // Sample a valid goal position
        std::array<double, 3> goal_pose;
        std::vector<double> random_state;
        double distance;
        bool within_bounds = false;
        
        // Keep sampling until we find a position within the desired distance range and environment bounds
        do {
            random_state = env.get_random_state();
            goal_pose = {random_state[0], random_state[1], 0.0};
            distance = std::sqrt(std::pow(object_state->position[0] - goal_pose[0], 2) + 
                               std::pow(object_state->position[1] - goal_pose[1], 2));
            
            // Check if within environment bounds
            within_bounds = goal_pose[0] >= x_min && goal_pose[0] <= x_max && 
                           goal_pose[1] >= y_min && goal_pose[1] <= y_max;
                              
        } while (distance < min_distance || distance > max_distance || !within_bounds);
        
        // Sample a random orientation
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> angle_dis(-M_PI, M_PI);
        double random_yaw = angle_dis(gen);
        std::array<double, 4> goal_quaternion = yaw_to_quaternion(random_yaw, true);
        
        // Create goal and set it in environment
        MujocoGoal goal;
        goal.position = goal_pose;
        goal.orientation = goal_quaternion;
        goal.size = object_info->size;
        goal.geom_type = object_info->geom_type;
        env.set_goal(goal);
        
        // Return the full goal state
        return {
            goal_pose[0], goal_pose[1], 0.0,
            goal_quaternion[0], goal_quaternion[1], goal_quaternion[2], goal_quaternion[3]
        };
    }

    /**
     * @brief Test if a specific action sequence can reach the goal
     * 
     * @param sequence Vector of action indices to execute
     * @param action_steps All available action steps
     * @param robot_global_goal Target position for the robot
     * @return bool True if sequence reaches the goal
     */
    bool testActionSequence(
        const std::vector<int>& sequence,
        const std::vector<std::unique_ptr<ActionStepMPC>>& action_steps,
        const std::array<double, 2>& robot_global_goal) {
        
        // Reset environment to initial state
        env.reset();
        
        // Execute each action in the sequence
        for (int action_idx : sequence) {
            ActionStepMPC* action_step = action_steps[action_idx].get();
            
            // Try to execute this action
            if (!executeActionIfReachable(action_step, robot_global_goal, false, "", true)) {
                // Action failed, sequence doesn't work
                return false;
            }
        }
        
        // After all actions, check if goal is reachable
        auto [all_edge_points, all_mid_points] = controller.get_object_edge_points_all();
        std::unordered_map<std::string, std::vector<std::array<double, 2>>> transformed_edge_points;
        updateTransformedEdgePoints(all_edge_points, transformed_edge_points);
        
        auto [wavefront, reachable_points, reachability_flags] = computeWavefront(transformed_edge_points, false);
        
        return wavefront_planner.is_goal_reachable(robot_global_goal, 0.3);
    }

    
};

} // namespace prx 