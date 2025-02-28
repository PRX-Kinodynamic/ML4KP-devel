#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "environment.hpp"
#include "push_controller.hpp"
#include <fstream>
#include <queue>
#include <chrono>
#include "namo_utility.hpp"
#include "motion_primitive_generator.hpp"

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
 * @brief Checks if a point is inside a rotated rectangle
 */
bool is_point_in_rotated_object(
    double point_x, 
    double point_y, 
    const NAMOEnvironment::ObjectInfo& obj,
    const NAMOEnvironment::ObjectState* obj_state=nullptr
) {
    // Check if state is provided or use object info directly
    if (obj_state == nullptr) {
        double angle = quaternion_to_yaw(obj.quaternion, true);

    // Translate point to object's local coordinates
    double local_x = point_x - obj.position[0];
    double local_y = point_y - obj.position[1];
    
    // Use utility function to rotate point
    auto rotated = rotate_point(point_x, point_y, obj.position[0], obj.position[1], -angle);
    
    // Check if point is inside the rectangle in local coordinates
    return std::abs(rotated[0] - obj.position[0]) <= obj.size[0] && 
           std::abs(rotated[1] - obj.position[1]) <= obj.size[1];
    }
    else {
        double angle = quaternion_to_yaw(obj_state->quaternion, true);

        // Translate point to object's local coordinates
        double local_x = point_x - obj_state->position[0];
        double local_y = point_y - obj_state->position[1];
            
        // Use utility function to rotate point
        auto rotated = rotate_point(point_x, point_y, obj_state->position[0], obj_state->position[1], -angle);
        
        return std::abs(rotated[0] - obj_state->position[0]) <= obj_state->size[0] && 
               std::abs(rotated[1] - obj_state->position[1]) <= obj_state->size[1];
    }   
}

/**
 * @brief Checks if a point is within a goal region
 */
bool is_point_in_goal_region(
    double point_x, 
    double point_y, 
    const std::array<double, 2>& goal_pos,
    double goal_size = 0.05
) {
    // Simple axis-aligned check since goals don't rotate
    return std::abs(point_x - goal_pos[0]) <= goal_size && 
           std::abs(point_y - goal_pos[1]) <= goal_size;
}

/**
 * @brief Computes a complete wavefront and checks goal reachability
 * 
 * @return tuple<grid, reachable_points, reachability_flags> Returns the wavefront grid, reachable points, and binary flags for each edge point
 */
std::tuple<
    std::vector<std::vector<int>>, 
    std::unordered_map<std::string, std::vector<std::array<double, 2>>>,
    std::unordered_map<std::string, std::vector<int>>
> 
compute_wavefront_with_goals(
    NAMOEnvironment& env,
    const std::vector<double>& start_pos,
    const std::unordered_map<std::string, std::vector<std::array<double, 2>>>& goal_positions,  
    double resolution = 0.1,
    const std::vector<double>& robot_size = {0.5, 0.5}
) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    auto bounds = env.get_environment_bounds();
    int grid_width = static_cast<int>((bounds[1] - bounds[0]) / resolution);
    int grid_height = static_cast<int>((bounds[3] - bounds[2]) / resolution);
    
    // Initialize grid and reachability map
    std::vector<std::vector<int>> grid(grid_width, std::vector<int>(grid_height, -1));
    std::unordered_map<std::string, std::set<std::pair<double, double>>> unique_points;
    
    // Mark obstacles (same as before)
    for (int x = 0; x < grid_width; x++) {
        for (int y = 0; y < grid_height; y++) {
            double world_x = bounds[0] + x * resolution;
            double world_y = bounds[2] + y * resolution;
            
            // Check static and movable objects
            for (const auto& obj : env.get_static_objects()) {
                NAMOEnvironment::ObjectInfo inflated_obj = obj;
                inflated_obj.size[0] += robot_size[0];
                inflated_obj.size[1] += robot_size[0];
                
                if (is_point_in_rotated_object(world_x, world_y, inflated_obj)) {
                    grid[x][y] = -2;
                    break;
                }
            }
            if (grid[x][y] != -2) {
                for (const auto& obj : env.get_movable_objects()) {
                    NAMOEnvironment::ObjectInfo inflated_obj = obj;
                    const NAMOEnvironment::ObjectState* inflated_obj_state = env.get_object_state(obj.name);

                    inflated_obj.size[0] += robot_size[0];
                    inflated_obj.size[1] += robot_size[0];
                    
                    if (is_point_in_rotated_object(world_x, world_y, inflated_obj, inflated_obj_state)) {
                        grid[x][y] = -2;
                        break;
                    }
                }
            }
        }
    }
    
    // Convert start to grid coordinates
    int start_x = static_cast<int>((start_pos[0] - bounds[0]) / resolution);
    int start_y = static_cast<int>((start_pos[1] - bounds[2]) / resolution);
    
    // BFS queue
    std::queue<std::pair<int, int>> q;
    q.push({start_x, start_y});
    grid[start_x][start_y] = 0;
    
    // Check if start position is in any goal region
    double start_world_x = bounds[0] + start_x * resolution;
    double start_world_y = bounds[2] + start_y * resolution;
    for (const auto& [obj_name, edge_points] : goal_positions) {
        for (const auto& point : edge_points) {
            if (is_point_in_goal_region(start_world_x, start_world_y, point)) {
                unique_points[obj_name].insert({point[0], point[1]});
            }
        }
    }
    
    // Directions for 8-connected grid
    std::vector<std::pair<int, int>> dirs = {
        {1,0}, {-1,0}, {0,1}, {0,-1},
        {1,1}, {1,-1}, {-1,1}, {-1,-1}
    };
    
    // Initialize reachability flags map - 12 points per object, all set to 0
    std::unordered_map<std::string, std::vector<int>> reachability_flags;
    for (const auto& [obj_name, edge_points] : goal_positions) {
        reachability_flags[obj_name] = std::vector<int>(12, 0);  // Initialize with 12 zeros
    }
    
    // Complete BFS with goal checking
    while (!q.empty()) {
        auto [x, y] = q.front();
        q.pop();
        
        for (const auto& [dx, dy] : dirs) {
            int nx = x + dx;
            int ny = y + dy;
            
            if (nx >= 0 && nx < grid_width && ny >= 0 && ny < grid_height && grid[nx][ny] == -1) {
                double world_x = bounds[0] + nx * resolution;
                double world_y = bounds[2] + ny * resolution;
                
                bool is_goal = false;
                // Check if this point is in any goal region
                for (const auto& [obj_name, edge_points] : goal_positions) {
                    for (size_t i = 0; i < edge_points.size(); i++) {
                        const auto& point = edge_points[i];
                        if (is_point_in_goal_region(world_x, world_y, point)) {
                            unique_points[obj_name].insert({point[0], point[1]});
                            reachability_flags[obj_name][i] = 1;  // Mark this edge point as reachable
                            is_goal = true;
                        }
                    }
                }
                
                grid[nx][ny] = is_goal ? 2 : 1;
                q.push({nx, ny});
            }
        }
    }
    
    // Convert sets to vectors for the return value
    std::unordered_map<std::string, std::vector<std::array<double, 2>>> reachable_points;
    for (const auto& [obj_name, point_set] : unique_points) {
        for (const auto& point : point_set) {
            reachable_points[obj_name].push_back({point.first, point.second});
        }
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    std::cout << "Wavefront computation took " << duration.count() << " milliseconds" << std::endl;
    
    return {grid, reachable_points, reachability_flags};
}

/**
 * @brief Save wavefront grid to a file
 */
void save_wavefront_to_file(
    const std::vector<std::vector<int>>& grid,
    const std::string& filename,
    const std::vector<double>& bounds,
    double resolution
) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file: " + filename);
    }
    
    // Write grid data
    for (size_t x = 0; x < grid.size(); x++) {
        for (size_t y = 0; y < grid[0].size(); y++) {
            // Convert grid coordinates to world coordinates
            double world_x = bounds[0] + x * resolution;
            double world_y = bounds[2] + y * resolution;
            file << world_x << " " << world_y << " " << grid[x][y] << "\n";
        }
    }
    file.close();
}

/**
 * @brief Transform a global pose to local frame relative to reference pose
 * 
 * @param reference_pose Reference pose [x, y, z, qw, qx, qy, qz]
 * @param global_pose Global pose to transform [x, y, z, qw, qx, qy, qz]
 * @return std::vector<double> Local pose [x, y, z, qw, qx, qy, qz]
 */
std::vector<double> transform_global_to_local_frame(
    const std::vector<double>& reference_pose,
    const std::vector<double>& global_pose)
{
    // Extract positions
    double ref_x = reference_pose[0];
    double ref_y = reference_pose[1];
    double ref_z = reference_pose[2];
    
    // Extract reference orientation as yaw
    std::array<double, 4> ref_quat = {
        reference_pose[3], reference_pose[4], 
        reference_pose[5], reference_pose[6]
    };
    double ref_yaw = quaternion_to_yaw(ref_quat, true);
    
    // Extract global position and orientation
    double global_x = global_pose[0];
    double global_y = global_pose[1];
    double global_z = global_pose[2];
    std::array<double, 4> global_quat = {
        global_pose[3], global_pose[4], 
        global_pose[5], global_pose[6]
    };
    double global_yaw = quaternion_to_yaw(global_quat, true);
    
    // Translate global position relative to reference
    double dx = global_x - ref_x;
    double dy = global_y - ref_y;
    
    // Rotate translated position by negative reference yaw
    double cos_ref = std::cos(-ref_yaw);
    double sin_ref = std::sin(-ref_yaw);
    double local_x = dx * cos_ref - dy * sin_ref;
    double local_y = dx * sin_ref + dy * cos_ref;
    
    // Calculate relative orientation (local yaw)
    double local_yaw = global_yaw - ref_yaw;
    
    // Normalize yaw to [-π, π]
    while (local_yaw > M_PI) local_yaw -= 2.0 * M_PI;
    while (local_yaw < -M_PI) local_yaw += 2.0 * M_PI;
    
    // Convert local yaw to quaternion
    std::array<double, 4> local_quat = yaw_to_quaternion(local_yaw, true);
    
    // Return local pose vector [x, y, z, qw, qx, qy, qz]
    return {
        local_x, local_y, global_z - ref_z,
        local_quat[0], local_quat[1], local_quat[2], local_quat[3]
    };
}

/**
 * @brief Print object pose information in local frame
 * 
 * @param primitive_idx Index of the current primitive
 * @param step_idx Step index within the primitive
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
        NAMOEnvironment env(xml_path, visualize);
        NAMOEnvironment::ObjectInfo robot_info = env.get_robot_info();

        auto bounds = env.get_environment_bounds();

        // Initialize the push controller with the environment
        std::string base_config_path = params["base_config_path"].as<std::string>();
        
        PushController controller(env, base_config_path, visualize);
        
        // Example robot start and goal positions (replace with actual positions)
        std::vector<double> robot_start = {robot_info.position[0], robot_info.position[1]};
        std::vector<double> robot_size = {0.05, 0.05}; //robot_info.size[0], robot_info.size[1]};  // For a 0.05 x 0.05 robot

        // Generate motion primitives for all movable objects
        int push_steps = 20;
        int control_steps = 500;
        double control_scale = 0.5;

        env.reset();

        bool visualize_primitives = false;

        controller.preprocess_all_motion_primitives(push_steps, control_steps, control_scale, visualize_primitives);

        std::unordered_map<std::string, std::vector<MotionPrimitive>> all_primitives = controller.get_all_primitives();

        auto [all_edge_points, all_mid_points] = controller.get_object_edge_points_all();

        std::unordered_map<std::string, std::vector<std::array<double, 2>>> transformed_edge_points;
        std::unordered_map<std::string, std::vector<std::array<double, 2>>> transformed_mid_points;
        
        for (const auto& [obj_name, edge_points] : all_edge_points) {
            auto obj_info = env.get_object_info(obj_name);
            std::cout << "obj_name: " << obj_name << std::endl;
            std::cout << "obj position: " << obj_info->position[0] << " " << obj_info->position[1] << std::endl;
            // std::cout << "obj quaternion: " << obj_info->quaternion[0] << " " << obj_info->quaternion[1] << " " << obj_info->quaternion[2] << " " << obj_info->quaternion[3] << std::endl;
            transformed_edge_points[obj_name] = MotionPrimitiveGenerator::transform_points(edge_points, obj_info->position, obj_info->quaternion);
            transformed_mid_points[obj_name] = MotionPrimitiveGenerator::transform_points(all_mid_points[obj_name], obj_info->position, obj_info->quaternion);
        }
        
        // Compute wavefront
        double resolution = 0.05;  // Adjust resolution as needed
        auto [wavefront, reachable_points, reachability_flags] = compute_wavefront_with_goals(env, robot_start, transformed_edge_points, resolution, robot_size);


        // save wavefront to file
        std::string output_path = "wavefront_data.txt";
        save_wavefront_to_file(wavefront, output_path, bounds, resolution);

        // You can now use reachability_flags to see which edge points are reachable for each object
        for (const auto& [obj_name, flags] : reachability_flags) {
            std::cout << "Object " << obj_name << " reachable edge points: ";
            for (int flag : flags) {
                std::cout << flag << " ";
            }
            std::cout << std::endl;
        }

        // for all objects, if the reachable points are not empty, create a list with object names
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

        // distance between current pos and goal pos
        std::array<double, 3> goal_pose;
        

        while (true) {
            auto random_state = env.get_random_state();
            goal_pose = {random_state[0], random_state[1],  0.0};
            double distance = std::sqrt(std::pow(random_object_info->position[0] - goal_pose[0], 2) + std::pow(random_object_info->position[1] - goal_pose[1], 2));
            if (distance > 0.5 && distance < 2.0) {
                break;
            }
        }

        
        // In your main function where you're testing the motion primitive
        std::vector<double> start_state = {
            random_object_info->position[0],
            random_object_info->position[1],
            0,  // z position if needed
            random_object_info->quaternion[0],
            random_object_info->quaternion[1],
            random_object_info->quaternion[2],
            random_object_info->quaternion[3]
        };

        // random number between -pi and pi
        std::uniform_real_distribution<> angle_dis(-M_PI, M_PI);
        double random_yaw = angle_dis(gen);

        std::array<double, 4> goal_quaternion = yaw_to_quaternion(random_yaw, true);
        std::vector<double> goal_state = {goal_pose[0], goal_pose[1], 0.0, goal_quaternion[0], goal_quaternion[1], goal_quaternion[2], goal_quaternion[3]};  // Your goal state

        // Get the allowed primitive indices from your reachability flags
        std::vector<int> allowed_indices;
        for (size_t i = 0; i < reachability_flags[random_object].size(); i++) {
            if (reachability_flags[random_object][i] == 1) {
                allowed_indices.push_back(i);
            }
        }

        // Compute the plan
        auto primitive_sequence = controller.compute_control_plan(
            random_object, 
            start_state, 
            goal_state,
            allowed_indices
        );

        MujocoGoal goal;
        goal.position = goal_pose;
        goal.orientation = goal_quaternion;
        goal.size = random_object_info->size;
        goal.geom_type = random_object_info->geom_type;
        env.set_goal(goal);

        env.reset();

        space_point_t control_point = env.get_control_space_point();
        
        // Open a file to record primitive execution states
        std::ofstream primitive_state_file("primitive_execution_states.txt");
        if (!primitive_state_file.is_open()) {
            std::cerr << "Error: Could not open primitive_execution_states.txt for writing" << std::endl;
        }

        // Open a file to record primitive execution states
        std::ofstream primitive_poses_file("primitive_poses.txt");
        if (!primitive_poses_file.is_open()) {
            std::cerr << "Error: Could not open primitive_poses.txt for writing" << std::endl;
        }
        
        // Use the sequence...
        if (!primitive_sequence.empty()) {
            // Save the initial state
            if (primitive_state_file.is_open()) {
                primitive_state_file << start_state[0] << " " << start_state[1] << " " << start_state[2] << " "
                                    << start_state[3] << " " << start_state[4] << " " << start_state[5] << " " 
                                    << start_state[6] << "\n";
            }
            
            int previous_edge_idx = -1;
            int ctr = 0;
            std::vector<double> original_state = start_state;
            
            for (int primitive_idx = 0; primitive_idx < primitive_sequence.size(); primitive_idx++) {
                const auto& primitive = primitive_sequence[primitive_idx];
                
                std::cout << "Executing primitive "
                          << " (edge_idx: " << primitive.edge_idx 
                          << ", push_steps: " << primitive.push_steps << ")\n";
                
                env.set_zero_velocity();
                for (int i = 0; i < 5; i++) {
                    env.step_simulation();
                }
                auto object_state = env.get_object_state(random_object);


                
                
                // std::cout << "object_state: " << object_state->position[0] << " " << object_state->position[1] << std::endl;
                // position and orientation of the random object with respect to the initial position and orientation, make sure the orientation transfromed correctly from the initial orientation
                auto push_points = MotionPrimitiveGenerator::transform_points(all_edge_points[random_object], object_state->position, object_state->quaternion);
                auto mid_points = MotionPrimitiveGenerator::transform_points(all_mid_points[random_object], object_state->position, object_state->quaternion);

                transformed_edge_points[random_object] = push_points;
                auto [wavefront_1, reachable_points_1, reachability_flags_1] = compute_wavefront_with_goals(env, robot_start, transformed_edge_points, resolution, robot_size);

                // save wavefront to file
                std::string output_path = "wavefront_data_1.txt";
                save_wavefront_to_file(wavefront_1, output_path, bounds, resolution);
                
                MotionPrimitiveGenerator::PushState push_state;
                
                for (int i = 0; i < primitive.push_steps; i++) {
                    if (i == 0) {
                        // for the push step, set the robot position to the push point
                        if (ctr == 0) {
                            env.set_robot_position(push_points[primitive.edge_idx]);
                        } else if (ctr > 0 && previous_edge_idx != primitive.edge_idx) {
                            env.set_robot_position(push_points[primitive.edge_idx]);
                        }
                        ctr++;
                        push_state.edge_idx = primitive.edge_idx;
                        previous_edge_idx = primitive.edge_idx;
                    }
                    env.set_zero_velocity();
                    env.step_simulation();
                    for (int j = 0; j < control_steps; j++) {
                        // update the push_point, mid_point, based on the current state of the selected random object
                        auto current_object_state = env.get_object_state(random_object);
                        auto current_push_points = MotionPrimitiveGenerator::transform_points(all_edge_points[random_object], current_object_state->position, current_object_state->quaternion);
                        auto current_mid_points = MotionPrimitiveGenerator::transform_points(all_mid_points[random_object], current_object_state->position, current_object_state->quaternion);

                        // update the push_state and generate new control
                        push_state.current_edge_point = current_push_points[primitive.edge_idx];
                        push_state.current_mid_point = current_mid_points[primitive.edge_idx];
                        auto control = MotionPrimitiveGenerator::compute_control1(push_state, control_scale);
                        control_point->at(0) = control[0];
                        control_point->at(1) = control[1];
                        env.step(control_point, 0.01);

                        current_object_state = env.get_object_state(random_object);
                        // std::cout << "current_object_state: " << current_object_state->position[0] << " " << current_object_state->position[1] << std::endl;
                    }
                    env.set_zero_velocity();
                    env.step_simulation();
                    
                    // std::cout << "push step current_object_state: " << current_object_state->position[0] << " " << current_object_state->position[1] << std::endl;
                }

                auto final_object_state = env.get_object_state(random_object);

                std::cout << "final_object_state: " << final_object_state->position[0] << " " << final_object_state->position[1] << std::endl;

                // After primitive execution, record the state
                if (primitive_state_file.is_open()) {
                    primitive_state_file << final_object_state->position[0] << " "
                                        << final_object_state->position[1] << " "
                                        << final_object_state->position[2] << " "
                                        << final_object_state->quaternion[0] << " "
                                        << final_object_state->quaternion[1] << " "
                                        << final_object_state->quaternion[2] << " "
                                        << final_object_state->quaternion[3] << "\n";
                }

                if (primitive_poses_file.is_open()) {
                    primitive_poses_file << primitive.pose[0] << " " << primitive.pose[1] << " " << primitive.pose[2] << " " << "\n";
                }
            }
            
            std::cout << std::endl;
            // Execute primitives...
        } else {
            std::cout << "No plan found" << std::endl;
            
            if (primitive_state_file.is_open()) {
                primitive_state_file.close();
            }

            if (primitive_poses_file.is_open()) {
                primitive_poses_file.close();
            }
        }

        // measure square root of the error in the pose
        auto final_object_state = env.get_object_state(random_object);
        double error_x = std::abs(goal_pose[0] - final_object_state->position[0]);
        double error_y = std::abs(goal_pose[1] - final_object_state->position[1]);
        double error = std::sqrt(error_x * error_x + error_y * error_y);

        // print final pose and goal pose
        std::cout << "final pose: " << final_object_state->position[0] << " " << final_object_state->position[1] << std::endl;
        std::cout << "goal pose: " << goal_pose[0] << " " << goal_pose[1] << std::endl;
        std::cout << "error: " << error << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}