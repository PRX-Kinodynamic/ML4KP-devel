#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "environment.hpp"
#include "push_controller.hpp"
#include <fstream>
#include <queue>
#include <chrono>
#include "namo_utility.hpp"
#include "motion_primitive_generator.hpp"

#include "wavefront_planner.hpp"
#include "namo_planner.hpp"
#include <nlohmann/json.hpp>
#include <unordered_set>
#include <filesystem>
#include <sstream>
#include <cstring>
#include <unistd.h>

using namespace prx;
using json = nlohmann::json;

/**
 * @brief Print detailed information about an object
 * 
 * @param obj Object information to print
 */
void print_object_info(const NAMOEnvironment::ObjectInfo& obj) {
    std::cout << "Object: " << obj.name << "\n";
    std::cout << "  Body ID: " << obj.body_id << "\n";
    std::cout << "  Geom ID: " << obj.geom_id << "\n";
    std::cout << "  Static: " << (obj.is_static ? "yes" : "no") << "\n";
    
    std::cout << "  Position: [" 
              << std::fixed << std::setprecision(3)
              << obj.position[0] << ", "
              << obj.position[1] << ", "
              << obj.position[2] << "]\n";
    
    std::cout << "  Size: ["
              << obj.size[0] << ", "
              << obj.size[1] << ", "
              << obj.size[2] << "]\n";
    
    std::cout << "  Quaternion: ["
              << obj.quaternion[0] << ", "
              << obj.quaternion[1] << ", "
              << obj.quaternion[2] << ", "
              << obj.quaternion[3] << "]\n";
    std::cout << "-------------------\n";
}


void print_local_pose(const std::vector<double>& local_pose) {
    double local_yaw = quaternion_to_yaw({
        local_pose[3], local_pose[4], local_pose[5], local_pose[6]
    }, true);
    
    std::cout << "  Local position: [" 
              << std::fixed << std::setprecision(3)
              << local_pose[0] << ", " << local_pose[1] << ", " << local_pose[2] << "]\n";
    std::cout << "  Local yaw (radians): " << local_yaw << "\n";
    std::cout << "  Local yaw (degrees): " << (local_yaw * 180.0 / M_PI) << "\n";
    std::cout << "-------------------\n";
}

/**
 * @brief Create directories helper function
 */
void createDirectories(const std::vector<std::filesystem::path>& paths) {
        for (auto& path : paths) {
            if (!std::filesystem::exists(path)) {
            std::filesystem::create_directories(path);
        }
    }
}

/**
 * @brief Set up motion primitives - either load or generate them
 */
void setupMotionPrimitives(
    const param_loader& params, 
    NAMOEnvironment& env, 
    PushController& controller,
    const std::string& xml_filename) {
    
    int push_steps = params["motion_primitives"]["push_steps"].as<int>();
    bool visualize_primitives = params["motion_primitives"]["visualize"].as<bool>();
    std::string base_config_path = params["motion_primitives"]["base_config_path"].as<std::string>();
    std::string primitives_path = params["motion_primitives"]["primitives_path"].as<std::string>();
    
    // Create primitives directory if needed
    std::filesystem::path primitives_dir(primitives_path);
    if (!std::filesystem::exists(primitives_dir)) {
        std::filesystem::create_directory(primitives_dir);
    }

    // Determine primitives filename based on environment
    std::string primitives_filename = primitives_path + "/" + xml_filename + ".json";
    
    // Try to load existing primitives

    

    // bool primitives_loaded = false;
    // if (std::filesystem::exists(primitives_filename)) {
    //     primitives_loaded = controller.load_primitives_from_json(primitives_filename);
    // }

    // bool reprocess_primitives = params["motion_primitives"]["reprocess_primitives"].as<bool>();
    // // Generate primitives if needed
    // if (!primitives_loaded || reprocess_primitives) {
    //     std::cout << "Generating new motion primitives..." << std::endl;
        
    //     // Preprocess environment to generate motion primitives
    //     controller.preprocess_all_motion_primitives(base_config_path, push_steps, visualize_primitives); 
        
    //     // Save motion primitives to JSON file
    //     controller.save_primitives_to_json(primitives_filename);
    //     std::cout << "Saved motion primitives to: " << primitives_filename << std::endl;
    // }

    std::cout << "Generating new motion primitives..." << std::endl;
    controller.preprocess_all_motion_primitives(base_config_path, push_steps, visualize_primitives); 
    std::cout << "Done generating motion primitives" << std::endl;
}

/**
 * @brief Generate a unique experiment ID based on timestamp and hostname
 * 
 * @return std::string Unique experiment ID
 */
std::string generateExperimentId() {
    // Get current time
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    
    // Format as string
    std::stringstream timestamp;
    timestamp << std::put_time(std::localtime(&time_t_now), "%Y%m%d_%H%M%S");
    
    // Get hostname
    char hostname[256];
    if (gethostname(hostname, sizeof(hostname)) != 0) {
        strcpy(hostname, "unknown");
    }
    
    // Create unique ID
    return timestamp.str() + "_" + hostname;
}

/**
 * @brief Main entry point for NAMO interface
 * 
 * Sets up the environment and provides a command-line interface for:
 * - Loading environment parameters
 * - Initializing the simulation
 * - Visualizing object states
 * - Testing motion primitives
 * 
 * @param argc Number of command line arguments
 * @param argv Command line arguments
 * @return int Exit code
 */

int convert_ch_to_int(char ch) {
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

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cout << "Usage: " << argv[0] << " <parameter_file_path>" << std::endl;
        return 1;
    }

    // Load parameters from YAML file
    param_loader params(argv[1]);
    
    // Initialize random seed if provided
    init_random(params["random_seed"].as<int>());

    bool diffusion_enabled = params["diffusion"]["enabled"].as<bool>();
    bool diffusion_goal_enabled = params["diffusion"]["goal_enabled"].as<bool>();
    bool evaluate_mode = params["evaluate"].as<bool>();
    int object_strategy = params["object_strategy"].as<int>();
    bool smoothing_enabled = params["smoothing_enabled"].as<bool>();
    std::string endpoint = params["endpoint"].as<std::string>();

    // Create necessary directories
    std::filesystem::path all_stats_dir("all_stats");
    std::filesystem::path wavefronts_dir("wavefronts");
    std::filesystem::path final_wavefronts_dir("sol_wavefronts");
    std::filesystem::path results_folder(params["results_folder"].as<std::string>());
    std::filesystem::path results_dir(results_folder);
    createDirectories({all_stats_dir, wavefronts_dir, final_wavefronts_dir, results_dir});
    std::string results_file_name = "results_" + std::string(diffusion_enabled ? "diffusion_object" : "random") + std::string(diffusion_goal_enabled ? "goal" : "") + ".txt";
    std::filesystem::path results_file = results_dir / results_file_name;
    std::ofstream file;
    if (evaluate_mode) {
        file.open(results_file.string(), std::ios::app);
    }
    
    // Extract XML filename for use in both try and catch blocks
    std::string xml_path = params["xml_path"].as<std::string>();
    std::filesystem::path xml_file_path(xml_path);
    std::string xml_filename = xml_file_path.stem().string();
    std::string parent_xml_filename = xml_file_path.parent_path().stem().string();
    bool is_one_env = params["one_env"].as<bool>();



    std::vector<std::unique_ptr<ActionStepMPC>> action_steps;
    
    try {
        // Environment setup
        bool visualize = params["visualize"].as<bool>();
        NAMOEnvironment env(xml_path, visualize);

        // Control parameters
        double distance_threshold = params["control_planner"]["distance_threshold"].as<double>();
        double angle_threshold = params["control_planner"]["angle_threshold"].as<double>();
        int mpc_steps_limit = params["control_planner"]["mpc_steps_limit"].as<int>();
        int best_first_expansion_limit = params["control_planner"]["best_first_expansion_limit"].as<int>();

        // Define get_distance lambda 
        auto get_distance = [](
            const std::vector<double>& state,
            const std::vector<double>& goal,
            const int symmetry_rotations
        ) -> double {
            // Position distance
            double dx = state[0] - goal[0];
            double dy = state[1] - goal[1];
            double pos_dist = std::sqrt(dx*dx + dy*dy);
            
            // Orientation distance with symmetry
            std::array<double, 4> q1 = yaw_to_quaternion(state[2], true);
            std::array<double, 4> q2 = yaw_to_quaternion(goal[2], true);
            double rot_dist = quaternion_distance_symmetric(q1, q2, symmetry_rotations, true);
            
            // Weight rotation in the distance calculation
            return pos_dist + 1.0 * rot_dist;
        };
        
        // Define is_goal_reached lambda
        auto is_goal_reached = [distance_threshold, angle_threshold](
            const std::vector<double>& state,
            const std::vector<double>& goal,
            const int symmetry_rotations
        ) -> bool {
            // Position check
            double dx = state[0] - goal[0];
            double dy = state[1] - goal[1];
            double distance = std::sqrt(dx*dx + dy*dy);
            
            // Orientation check with symmetry
            std::array<double, 4> q1 = yaw_to_quaternion(state[2], true);
            std::array<double, 4> q2 = yaw_to_quaternion(goal[2], true);
            double rot_dist = quaternion_distance_symmetric(q1, q2, symmetry_rotations, true);
            
            return distance < distance_threshold && rot_dist < angle_threshold;
        };

        // Initialize the push controller with the environment
        int control_steps = params["motion_primitives"]["control_steps"].as<int>();
        double control_scale = params["motion_primitives"]["control_scale"].as<double>();
        PushController controller(env, visualize, control_steps, control_scale, 
                                 get_distance, is_goal_reached, 
                                 mpc_steps_limit, best_first_expansion_limit);
        
        // Get robot information
        NAMOEnvironment::ObjectInfo robot_info = env.get_robot_info();
        std::vector<double> robot_start = {robot_info.position[0], robot_info.position[1]};
        std::vector<double> robot_size = {robot_info.size[0], robot_info.size[1]};
        
        env.reset();
        
        // Load or generate motion primitives
        if (is_one_env) {
            setupMotionPrimitives(params, env, controller, parent_xml_filename);
        }
        else {
            setupMotionPrimitives(params, env, controller, parent_xml_filename + "_" + xml_filename);
        }
        
        // Initialize wavefront planner
        double resolution = params["wavefront_planner"]["resolution"].as<double>();
        WavefrontPlanner wavefront_planner(resolution, env, robot_size);

        // wavefront_planner.save_wavefront_to_file("test.txt");
        
        // Get robot goal position
        std::array<double, 2> robot_global_goal = params["robot_goal"].as<std::array<double, 2>>();
        env.set_robot_goal(robot_global_goal);
        
        // Create the NAMO planner
        NAMOPlanner planner(env, controller, wavefront_planner, object_strategy, diffusion_enabled, diffusion_goal_enabled, endpoint);
        
        // Run the main planning loop
        int total_iter = params["total_iter"].as<int>();
        bool success = planner.performPlanningLoop(robot_global_goal, total_iter, wavefronts_dir, action_steps);
        
        if (success) {
            
            std::cout << "Successfully found a plan to reach the goal!" << std::endl;
            if (evaluate_mode) {
                std::string buffer = xml_filename + "," + "1" + "," + std::to_string(action_steps.size());
                file << buffer;
            }
            // Optimize action sequence

    
            std::vector<std::vector<int>> optimized_sequences_final_state;
            std::vector<std::vector<int>> optimized_sequences_goal_state;
            if (smoothing_enabled) {
                // action_steps > 5, then only use the last 5 actions to optimize the final state
                if (action_steps.size() > 5) {
                    std::vector<std::unique_ptr<ActionStepMPC>> action_steps_last_5;
                    for (int i = action_steps.size() - 5; i < action_steps.size(); i++) {
                        std::unique_ptr<ActionStepMPC> action_step_ptr = std::make_unique<ActionStepMPC>();
                        action_step_ptr->object_name = action_steps[i]->object_name;
                        action_step_ptr->goal_state = action_steps[i]->goal_state;
                        action_step_ptr->final_state = action_steps[i]->final_state;
                        action_steps_last_5.push_back(std::move(action_step_ptr));
                    }
                    auto subset_sequences = planner.optimizeActionSequence2(action_steps_last_5, robot_global_goal, final_wavefronts_dir);
                    
                    // Map indices back to original action_steps
                    for (auto& seq : subset_sequences) {
                        for (auto& idx : seq) {
                            idx += (action_steps.size() - 5);  // Adjust to original indices
                        }
                    }
                    optimized_sequences_final_state = subset_sequences;
                }
                else{
                    optimized_sequences_final_state = planner.optimizeActionSequence2(action_steps, robot_global_goal, final_wavefronts_dir);
                }

                if (optimized_sequences_final_state.size() == 0) {
                    std::cout << "No optimized action sequences found." << std::endl;
                    if (evaluate_mode) {
                        std::string buffer = "," + std::to_string(action_steps.size());
                        file << buffer;
                    }
                }
                std::cout << "Found " << optimized_sequences_final_state.size() << " optimized final state action sequences." << std::endl;
            
                
                optimized_sequences_goal_state = planner.optimizeActionSequence(action_steps, robot_global_goal, final_wavefronts_dir);

                if (optimized_sequences_goal_state.size() == 0) {
                    std::cout << "No optimized action sequences found." << std::endl;
                    if (evaluate_mode) {
                        std::string buffer = "," + std::to_string(action_steps.size());
                        file << buffer;
                    }
                }   

                std::cout << "Found " << optimized_sequences_goal_state.size() << " optimized goal state action sequences." << std::endl;
                

                if (evaluate_mode) {
                    for (const auto& seq : optimized_sequences_final_state) {
                        std::string buffer = "," + std::to_string(seq.size());
                        file << buffer;
                    }
                    for (const auto& seq : optimized_sequences_goal_state) {
                        std::string buffer = "," + std::to_string(seq.size());
                        file << buffer;
                    }
                    file << "\n";
                }
            }
            else{
                std::string buffer = ",\n";
                file << buffer;
            }
            // else{
            //     optimized_sequences = action_steps;
            // }


            if (optimized_sequences_final_state.size() == 0 && optimized_sequences_goal_state.size() == 0) {
                std::cout << "No optimized action sequences found." << std::endl;
                if (evaluate_mode) {
                    std::string buffer = "," + std::to_string(action_steps.size());
                    file << buffer;
                }
            }
            
            // Check if data collection is enabled in parameters
            bool collect_data = params["data_collection"]["enabled"].as<bool>();  // Default to false if not specified
            
            if (smoothing_enabled && collect_data) {
                // Create data collection directory
                std::filesystem::path data_dir = params["data_collection"]["output_dir"].as<std::string>();
                if (!std::filesystem::exists(data_dir)) {
                    std::filesystem::create_directories(data_dir);
                }
                
                // Generate experiment ID with configurable prefix
                std::string experiment_prefix = "";
                
                experiment_prefix = params["data_collection"]["run_id"].as<std::string>() + "_";
                
                // Combine prefix with unique process identifier
                std::string experiment_id = experiment_prefix + "final_state_" + generateExperimentId();
                
                // Collect and store state-action pairs
                int collected_points = planner.collectStateActionPairs(
                    optimized_sequences_final_state, action_steps, data_dir, experiment_id, true);

                experiment_id = experiment_prefix + "goal_state_" + generateExperimentId();

                collected_points += planner.collectStateActionPairs(
                    optimized_sequences_goal_state, action_steps, data_dir, experiment_id, false);
                
                std::cout << "Collected " << collected_points << " state-action pairs." << std::endl;
                std::cout << "Data stored in " << data_dir << " with experiment ID: " << experiment_id << std::endl;
            } else {
                std::cout << "Data collection is disabled in configuration." << std::endl;
            }
        } else {
            std::cout << "Could not find a plan to reach the goal within " << total_iter << " iterations." << std::endl;
            if (evaluate_mode) {    
                std::string buffer = xml_filename + ",0,-1,-1\n";
                file << buffer;
            }
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        file.close();
        return 1;
    }
    file.close();
    return 0;
}