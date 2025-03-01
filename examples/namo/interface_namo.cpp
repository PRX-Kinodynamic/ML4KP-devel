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

using namespace prx;

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

/**
 * @brief Print object pose information in local frame
 * 
 * @param local_pose Local pose [x, y, z, qw, qx, qy, qz]
 */
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


int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cout << "Usage: " << argv[0] << " <parameter_file_path>" << std::endl;
        return 1;
    }

    // Load parameters from YAML file
    param_loader params(argv[1]);
    
    // Initialize random seed if provided
    init_random(params["random_seed"].as<int>());

    // Create environment
    std::string xml_path = params["xml_path"].as<std::string>();
    bool visualize = params["visualize"].as<bool>();
    
    try {

        // setup starts
        NAMOEnvironment env(xml_path, visualize);
        auto bounds = env.get_environment_bounds();

        double distance_threshold = params["control_planner"]["distance_threshold"].as<double>();
        double angle_threshold = params["control_planner"]["angle_threshold"].as<double>();
        int mpc_steps_limit = params["control_planner"]["mpc_steps_limit"].as<int>();
        int best_first_expansion_limit = params["control_planner"]["best_first_expansion_limit"].as<int>();

        // Define get_distance lambda (formerly heuristic)
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
            
            // Increase rotation weight (adjust this value as needed)
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

        PushController controller(env, visualize, get_distance, is_goal_reached, mpc_steps_limit, best_first_expansion_limit);
        
        // Example robot start and goal positions (replace with actual positions)
        NAMOEnvironment::ObjectInfo robot_info = env.get_robot_info();
        std::vector<double> robot_start = {robot_info.position[0], robot_info.position[1]};
        std::vector<double> robot_size = {0.05, 0.05}; //robot_info.size[0], robot_info.size[1]};  // For a 0.05 x 0.05 robot
        
        env.reset();
        
        // Generate motion primitives for all movable objects
        int push_steps = params["motion_primitives"]["push_steps"].as<int>();
        int control_steps = params["motion_primitives"]["control_steps"].as<int>();
        double control_scale = params["motion_primitives"]["control_scale"].as<double>();
        bool visualize_primitives = params["motion_primitives"]["visualize"].as<bool>();
        std::string base_config_path = params["motion_primitives"]["base_config_path"].as<std::string>();
        
        // preprocess environment to generate motion primitives. In addition we process transformed edge points in the world frame
        controller.preprocess_all_motion_primitives(base_config_path, push_steps, control_steps, control_scale, visualize_primitives); 
        auto [all_edge_points, all_mid_points] = controller.get_object_edge_points_all();
        std::unordered_map<std::string, std::vector<std::array<double, 2>>> transformed_edge_points;
        for (const auto& [obj_name, edge_points] : all_edge_points) {
            auto obj_info = env.get_object_info(obj_name);
            transformed_edge_points[obj_name] = MotionPrimitiveGenerator::transform_points(edge_points, obj_info->position, obj_info->quaternion);
        }

        double resolution = 0.05;  // Adjust resolution as needed
        
        // setup ends
       
        // task planning functionality starts
        std::vector<double> robot_global_goal = env.get_random_goal_configuration();

        // Compute wavefront to find reachable edge points for each object given the robot start position and size
        auto [wavefront, reachable_points, reachability_flags] = compute_wavefront_with_goals(resolution, env, robot_start, robot_size, transformed_edge_points);
        // save wavefront to file
        std::string output_path = "wavefront_data.txt";
        save_wavefront_to_file(wavefront, output_path, bounds, resolution);

        // list of reachable objects
        std::vector<std::string> reachable_objects;
        for (const auto& [obj_name, points] : reachable_points) {
            if (!points.empty()) {
                reachable_objects.push_back(obj_name);
            }
        }

        // select a reachable object at random
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> idx_dis(0, reachable_objects.size() - 1);
        int random_index = idx_dis(gen);
        std::string random_object = reachable_objects[random_index];
        auto random_object_info = env.get_object_info(random_object);

        // For the selected object, get the allowed primitive indices from your reachability flags
        std::vector<int> allowed_indices;
        for (size_t i = 0; i < reachability_flags[random_object].size(); i++) {
            if (reachability_flags[random_object][i] == 1) {
                allowed_indices.push_back(i);
            }
        }

        std::vector<double> start_state = {random_object_info->position[0], random_object_info->position[1], 0.0, random_object_info->quaternion[0], random_object_info->quaternion[1], random_object_info->quaternion[2], random_object_info->quaternion[3]};
        std::vector<double> goal_state = env.set_goal_configuration(random_object);


        // controller functionality starts
        // call the controller with execute_push_action(random_object, allowed_indices, goal_state)
        bool controller_success = controller.execute_push_action(random_object, allowed_indices, goal_state); // The controller has access to the state of the environment
        

        

        // measure square root of the error in the pose
        auto final_object_state = env.get_object_state(random_object);
        double error_x = std::abs(goal_state[0] - final_object_state->position[0]);
        double error_y = std::abs(goal_state[1] - final_object_state->position[1]);
        double error = std::sqrt(error_x * error_x + error_y * error_y);

        std::array<double, 4> goal_quat = {goal_state[3], goal_state[4], goal_state[5], goal_state[6]};
        std::array<double, 4> final_quat = {final_object_state->quaternion[0], final_object_state->quaternion[1], final_object_state->quaternion[2], final_object_state->quaternion[3]};
        double error_quaternion = quaternion_distance_symmetric(goal_quat, final_quat, random_object_info->symmetry_rotations, true);

        // print final pose and goal pose
        std::cout << "final pose: " << final_object_state->position[0] << " " << final_object_state->position[1] << std::endl;
        std::cout << "goal pose: " << goal_state[0] << " " << goal_state[1] << std::endl;
        std::cout << "error: " << error << std::endl;
        std::cout << "error_quaternion: " << error_quaternion << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}