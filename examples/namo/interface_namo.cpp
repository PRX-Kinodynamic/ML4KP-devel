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
#include <iomanip>

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
            std::filesystem::create_directory(path);
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
    bool primitives_loaded = false;
    if (std::filesystem::exists(primitives_filename)) {
        primitives_loaded = controller.load_primitives_from_json(primitives_filename);
    }

    bool reprocess_primitives = params["motion_primitives"]["reprocess_primitives"].as<bool>();
    // Generate primitives if needed
    if (!primitives_loaded || reprocess_primitives) {
        std::cout << "Generating new motion primitives..." << std::endl;
        
        // Preprocess environment to generate motion primitives
        controller.preprocess_all_motion_primitives(base_config_path, push_steps, visualize_primitives); 
        
        // Save motion primitives to JSON file
        controller.save_primitives_to_json(primitives_filename);
        std::cout << "Saved motion primitives to: " << primitives_filename << std::endl;
    }
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

    // Create necessary directories
    std::filesystem::path all_stats_dir("all_stats");
    std::filesystem::path wavefronts_dir("wavefronts");
    std::filesystem::path final_wavefronts_dir("sol_wavefronts");
    createDirectories({all_stats_dir, wavefronts_dir, final_wavefronts_dir});
    
    // Extract XML filename for use in both try and catch blocks
    std::string xml_path = params["xml_path"].as<std::string>();
    std::filesystem::path xml_file_path(xml_path);
    std::string xml_filename = xml_file_path.stem().string();

    std::vector<std::unique_ptr<ActionStep>> action_steps;
    
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
        setupMotionPrimitives(params, env, controller, xml_filename);
        
        // Initialize wavefront planner
        double resolution = params["wavefront_planner"]["resolution"].as<double>();
        WavefrontPlanner wavefront_planner(resolution, env, robot_size);
        
        // Get robot goal position
        std::vector<double> robot_global_goal = params["robot_goal"].as<std::vector<double>>();
        
        // Create the NAMO planner
        NAMOPlanner planner(env, controller, wavefront_planner);
        
        // Run the main planning loop
        int total_iter = params["total_iter"].as<int>();
        bool success = planner.performPlanningLoop(robot_global_goal, total_iter, wavefronts_dir, action_steps);
        
        if (success) {
            std::cout << "Successfully found a plan to reach the goal!" << std::endl;
            
            // Optimize action sequence`
            std::vector<std::vector<int>> optimized_sequences = planner.optimizeActionSequence(
                action_steps, robot_global_goal, final_wavefronts_dir);
            
            std::cout << "Found " << optimized_sequences.size() << " optimized action sequences." << std::endl;
            
            // Check if data collection is enabled in parameters
            bool collect_data = params["data_collection"]["enabled"].as<bool>();  // Default to false if not specified
            
            if (collect_data) {
                // Create data collection directory
                std::filesystem::path data_dir = params["data_collection"]["output_dir"].as<std::string>();
                if (!std::filesystem::exists(data_dir)) {
                    std::filesystem::create_directories(data_dir);
                }
                
                // Generate experiment ID with configurable prefix
                std::string experiment_prefix = "";
                
                experiment_prefix = params["data_collection"]["run_id"].as<std::string>() + "_";
                
                // Combine prefix with unique process identifier
                std::string experiment_id = experiment_prefix + generateExperimentId();
                
                // Collect and store state-action pairs
                int collected_points = planner.collectStateActionPairs(
                    optimized_sequences, action_steps, data_dir, experiment_id);
                
                std::cout << "Collected " << collected_points << " state-action pairs." << std::endl;
                std::cout << "Data stored in " << data_dir << " with experiment ID: " << experiment_id << std::endl;
            } else {
                std::cout << "Data collection is disabled in configuration." << std::endl;
            }
        } else {
            std::cout << "Could not find a plan to reach the goal within " << total_iter << " iterations." << std::endl;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}



