#ifndef WAVEFRONT_HPP
#define WAVEFRONT_HPP

# pragma once
#include <vector>
#include <array>
#include <queue>
#include <set>
#include <unordered_map>
#include <string>
#include <fstream>
#include <stdexcept>
#include <chrono>
#include "environment.hpp"
#include "namo_utility.hpp"

namespace prx {

/**
 * @brief Checks if a point is inside a rotated rectangle
 */
bool is_point_in_rotated_object(
    double point_x, 
    double point_y, 
    const NAMOEnvironment::ObjectInfo& obj,
    const NAMOEnvironment::ObjectState* obj_state=nullptr
);

/**
 * @brief Checks if a point is within a goal region
 */
bool is_point_in_goal_region(
    double point_x, 
    double point_y, 
    const std::array<double, 2>& goal_pos,
    double goal_size = 0.05
);

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
    double resolution,
    NAMOEnvironment& env,
    const std::vector<double>& start_pos,
    const std::vector<double>& robot_size,
    const std::unordered_map<std::string, std::vector<std::array<double, 2>>>& goal_positions
);

/**
 * @brief Save wavefront grid to a file
 */
void save_wavefront_to_file(
    const std::vector<std::vector<int>>& grid,
    const std::string& filename,
    const std::vector<double>& bounds,
    double resolution
);

// Implementations

/**
 * @brief Checks if a point is inside a rotated rectangle
 */
bool is_point_in_rotated_object(
    double point_x, 
    double point_y, 
    const NAMOEnvironment::ObjectInfo& obj,
    const NAMOEnvironment::ObjectState* obj_state
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
    double goal_size
) {
    // Simple axis-aligned check since goals don't rotate
    return std::abs(point_x - goal_pos[0]) <= goal_size && 
           std::abs(point_y - goal_pos[1]) <= goal_size;
}

/**
 * @brief Computes a complete wavefront and checks goal reachability
 */
std::tuple<
    std::vector<std::vector<int>>, 
    std::unordered_map<std::string, std::vector<std::array<double, 2>>>,
    std::unordered_map<std::string, std::vector<int>>
> 
compute_wavefront_with_goals(
    double resolution,
    NAMOEnvironment& env,
    const std::vector<double>& start_pos,
    const std::vector<double>& robot_size,
    const std::unordered_map<std::string, std::vector<std::array<double, 2>>>& goal_positions
) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    auto bounds = env.get_environment_bounds();
    int grid_width = static_cast<int>((bounds[1] - bounds[0]) / resolution);
    int grid_height = static_cast<int>((bounds[3] - bounds[2]) / resolution);
    
    // Initialize grid and reachability map
    std::vector<std::vector<int>> grid(grid_width, std::vector<int>(grid_height, -1));
    std::unordered_map<std::string, std::set<std::pair<double, double>>> unique_points;
    
    // Mark obstacles
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

}  // namespace prx

#endif // WAVEFRONT_HPP 