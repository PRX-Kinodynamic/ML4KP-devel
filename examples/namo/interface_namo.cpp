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
    const NAMOEnvironment::ObjectInfo& obj
) {
    // Convert quaternion to 2D rotation angle using utility function
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
                    inflated_obj.size[0] += robot_size[0];
                    inflated_obj.size[1] += robot_size[0];
                    
                    if (is_point_in_rotated_object(world_x, world_y, inflated_obj)) {
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

        controller.preprocess_all_motion_primitives(push_steps, control_steps, control_scale);


        std::unordered_map<std::string, std::vector<MotionPrimitive>> all_primitives = controller.get_all_primitives();

        auto [goal_positions, mid_points] = controller.get_object_edge_points_all();
        // Compute wavefront
        double resolution = 0.05;  // Adjust resolution as needed
        auto [wavefront, reachable_points, reachability_flags] = compute_wavefront_with_goals(env, robot_start, goal_positions, resolution, robot_size);


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
        std::vector<double> goal_pose;

        while (true) {
            goal_pose = env.get_random_state();
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

        std::array<double, 4> goal_quaternion = yaw_to_quaternion(random_yaw);
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

        // Use the sequence...
        if (!primitive_sequence.empty()) {
            std::cout << "Found plan with " << primitive_sequence.size() << " primitives" << std::endl;
            // Execute primitives...
        } else {
            std::cout << "No plan found" << std::endl;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}