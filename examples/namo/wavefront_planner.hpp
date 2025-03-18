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

// Forward declarations of functions moved into the class
bool is_point_in_rotated_object(
    double point_x, 
    double point_y, 
    const NAMOEnvironment::ObjectInfo& obj,
    const NAMOEnvironment::ObjectState* obj_state=nullptr
);

bool is_point_in_goal_region(
    double point_x, 
    double point_y, 
    const std::array<double, 2>& goal_pos,
    double goal_size = 0.05
);

// Keep this function for backward compatibility
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

void save_wavefront_to_file(
    const std::vector<std::vector<int>>& grid,
    const std::string& filename,
    const std::vector<double>& bounds,
    double resolution
);

bool is_goal_reachable(
    const std::vector<std::vector<int>>& grid,
    const std::vector<double>& goal_pos,
    NAMOEnvironment& env,
    double resolution,
    double goal_size = 0.05
);

/**
 * @brief WavefrontPlanner class for efficient grid computation and reuse
 */
class WavefrontPlanner {
private:
    double resolution;
    std::vector<double> bounds;
    int grid_width;
    int grid_height;
    std::vector<std::vector<int>> static_grid;  // Grid with only static obstacles
    std::vector<std::vector<int>> full_grid;    // Grid with all obstacles
    std::vector<std::vector<int>> piecewise_grid;    // Grid with all obstacles
    std::vector<double> robot_size;
    
    // BFS directions for 8-connected grid
    std::vector<std::pair<int, int>> dirs = {
        {1,0}, {-1,0}, {0,1}, {0,-1},
        {1,1}, {1,-1}, {-1,1}, {-1,-1}
    };
    
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
            
            return std::abs(rotated[0] - obj_state->position[0]) <= obj.size[0] && 
                   std::abs(rotated[1] - obj_state->position[1]) <= obj.size[1];
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

public:
    /**
     * @brief Construct a new Wavefront Planner object
     * 
     * @param resolution Grid resolution
     * @param env Environment
     * @param robot_size Size of the robot [width, height]
     */
    WavefrontPlanner(double resolution, NAMOEnvironment& env, const std::vector<double>& robot_size)
        : resolution(resolution), robot_size(robot_size)
    {
        // Initialize grid dimensions based on environment bounds
        bounds = env.get_environment_bounds();
        grid_width = static_cast<int>((bounds[1] - bounds[0]) / resolution);
        grid_height = static_cast<int>((bounds[3] - bounds[2]) / resolution);
        
        // Allocate memory for grids
        static_grid.resize(grid_width, std::vector<int>(grid_height, -1));
        full_grid.resize(grid_width, std::vector<int>(grid_height, -1));
        piecewise_grid.resize(grid_width, std::vector<int>(grid_height, -1));
        
        // Precompute static obstacles grid
        for (int x = 0; x < grid_width; x++) {
            for (int y = 0; y < grid_height; y++) {
                double world_x = bounds[0] + x * resolution;
                double world_y = bounds[2] + y * resolution;
                
                // Check static objects
                for (const auto& obj : env.get_static_objects()) {
                    NAMOEnvironment::ObjectInfo inflated_obj = obj;
                    
                    inflated_obj.size[0] += robot_size[0];
                    inflated_obj.size[1] += robot_size[0];
                    
                    if (is_point_in_rotated_object(world_x, world_y, inflated_obj)) {
                        static_grid[x][y] = -2;
                        piecewise_grid[x][y] = -2;
                        break;
                    }
                }
            }
        }
    }
    
    /**
     * @brief Compute wavefront from given start position with preallocated grids
     * 
     * @param env Environment with current object states
     * @param start_pos Start position [x, y]
     * @param goal_positions Goal positions for reachability checking
     * @return tuple<grid, reachable_points, reachability_flags>
     */
    std::tuple<
        std::vector<std::vector<int>>, 
        std::unordered_map<std::string, std::vector<std::array<double, 2>>>,
        std::unordered_map<std::string, std::vector<int>>
    > 
    compute_wavefront(
        NAMOEnvironment& env,
        const std::vector<double>& start_pos,
        const std::unordered_map<std::string, std::vector<std::array<double, 2>>>& goal_positions 
    ) {
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // Copy static grid to full grid
        full_grid = static_grid;
        
        // Update grid with movable objects only
        for (int x = 0; x < grid_width; x++) {
            for (int y = 0; y < grid_height; y++) {
                if (full_grid[x][y] == -2) continue; // Skip cells that are already obstacles
                
                double world_x = bounds[0] + x * resolution;
                double world_y = bounds[2] + y * resolution;
                
                // Check only movable objects
                for (const auto& obj : env.get_movable_objects()) {
                    NAMOEnvironment::ObjectInfo inflated_obj = obj;
                    const NAMOEnvironment::ObjectState* inflated_obj_state = env.get_object_state(obj.name);
                    inflated_obj.size[0] += robot_size[0];
                    inflated_obj.size[1] += robot_size[0];
                    
                    if (is_point_in_rotated_object(world_x, world_y, inflated_obj, inflated_obj_state)) {
                        full_grid[x][y] = -2;
                        break;
                    }
                }
            }
        }
        
        // Convert start to grid coordinates
        int start_x = static_cast<int>((start_pos[0] - bounds[0]) / resolution);
        int start_y = static_cast<int>((start_pos[1] - bounds[2]) / resolution);
        
        // Reset grid values for BFS (keeping obstacles as -2)
        for (int x = 0; x < grid_width; x++) {
            for (int y = 0; y < grid_height; y++) {
                if (full_grid[x][y] != -2) {
                    full_grid[x][y] = -1;
                }
            }
        }
        
        // BFS queue
        std::queue<std::pair<int, int>> q;
        q.push({start_x, start_y});
        full_grid[start_x][start_y] = 0;
        
        // Initialize reachable points
        std::unordered_map<std::string, std::set<std::pair<double, double>>> unique_points;
        
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
        
        // Initialize reachability flags map
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
                
                if (nx >= 0 && nx < grid_width && ny >= 0 && ny < grid_height && full_grid[nx][ny] == -1) {
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
                    
                    full_grid[nx][ny] = is_goal ? 2 : 1;
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
        // std::cout << "Wavefront computation took " << duration.count() << " milliseconds" << std::endl;
        
        return {full_grid, reachable_points, reachability_flags};
    }


    void reset_piecewise_grid(NAMOEnvironment& env) {
        piecewise_grid = static_grid;

        // Update grid with movable objects only
        for (int x = 0; x < grid_width; x++) {
            for (int y = 0; y < grid_height; y++) {
                if (piecewise_grid[x][y] == -2) continue; // Skip cells that are already obstacles
                
                double world_x = bounds[0] + x * resolution;
                double world_y = bounds[2] + y * resolution;
                
                // Check only movable objects
                for (const auto& obj : env.get_movable_objects()) {
                    NAMOEnvironment::ObjectInfo inflated_obj = obj;
                    const NAMOEnvironment::ObjectState* inflated_obj_state = env.get_object_state(obj.name);
                    inflated_obj.size[0] += robot_size[0];
                    inflated_obj.size[1] += robot_size[0];
                    
                    if (is_point_in_rotated_object(world_x, world_y, inflated_obj, inflated_obj_state)) {
                        piecewise_grid[x][y] = -2;
                        break;
                    }
                }
            }
        }
    }

    std::tuple<
        std::vector<std::vector<int>>, 
        std::unordered_map<std::string, std::vector<std::array<double, 2>>>,
        std::unordered_map<std::string, std::vector<int>>,
        bool
    > 
    compute_piecewise_wavefront(
        NAMOEnvironment& env,
        const std::vector<double>& start_pos,
        const std::unordered_map<std::string, std::vector<std::array<double, 2>>>& goal_positions,
        int piece_number
    ) {
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // Convert start to grid coordinates
        int start_x = static_cast<int>((start_pos[0] - bounds[0]) / resolution);
        int start_y = static_cast<int>((start_pos[1] - bounds[2]) / resolution);

        if (piecewise_grid[start_x][start_y] != -1) {
            return {piecewise_grid, {}, {}, false};
        }
        
        // Reset grid values for BFS (keeping obstacles as -2)
        // for (int x = 0; x < grid_width; x++) {
        //     for (int y = 0; y < grid_height; y++) {
        //         if (piecewise_grid[x][y] != -2) {
        //             piecewise_grid[x][y] = -1;
        //         }
        //     }
        // }
        
        // BFS queue
        std::queue<std::pair<int, int>> q;
        q.push({start_x, start_y});
        piecewise_grid[start_x][start_y] = 0;
        
        // Initialize reachable points
        std::unordered_map<std::string, std::set<std::pair<double, double>>> unique_points;
        
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
        
        // Initialize reachability flags map
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
                
                if (nx >= 0 && nx < grid_width && ny >= 0 && ny < grid_height && piecewise_grid[nx][ny] == -1) {
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
                    
                    piecewise_grid[nx][ny] = piece_number;
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
        // std::cout << "Wavefront computation took " << duration.count() << " milliseconds" << std::endl;
        
        return {piecewise_grid, reachable_points, reachability_flags, true};
    }






    void reset_grid(NAMOEnvironment& env) {

        full_grid = static_grid;
        
        // Update grid with movable objects only
        for (int x = 0; x < grid_width; x++) {
            for (int y = 0; y < grid_height; y++) {
                if (full_grid[x][y] == -2) continue; // Skip cells that are already obstacles
                
                double world_x = bounds[0] + x * resolution;
                double world_y = bounds[2] + y * resolution;
                
                // Check only movable objects
                for (const auto& obj : env.get_movable_objects()) {
                    NAMOEnvironment::ObjectInfo inflated_obj = obj;
                    const NAMOEnvironment::ObjectState* inflated_obj_state = env.get_object_state(obj.name);
                    inflated_obj.size[0] += robot_size[0];
                    inflated_obj.size[1] += robot_size[0];
                    
                    if (is_point_in_rotated_object(world_x, world_y, inflated_obj, inflated_obj_state)) {
                        full_grid[x][y] = -2;
                        break;
                    }
                }
            }
        }
        
    }

    std::vector<std::vector<int>>
    compute_wavefront_within_radius(
        NAMOEnvironment& env,
        const std::vector<double>& start_pos,
        double radius
    ) {
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // Copy static grid to full grid
        // full_grid = static_grid;
        
        // reset_grid(env);
        
        // Convert start to grid coordinates
        int start_x = static_cast<int>((start_pos[0] - bounds[0]) / resolution);
        int start_y = static_cast<int>((start_pos[1] - bounds[2]) / resolution);
        
        // Reset grid values for BFS (keeping obstacles as -2)
        for (int x = 0; x < grid_width; x++) {
            for (int y = 0; y < grid_height; y++) {
                if ((full_grid[x][y] == -2) || (full_grid[x][y] == 1)) {
                    continue;
                }
                full_grid[x][y] = -1;
            }
        }
        
        // check if start position is in obstacle
        if (full_grid[start_x][start_y] == -2) {
            return full_grid;
        }

        // BFS queue
        std::queue<std::tuple<int, int, int>> q;
        q.push({start_x, start_y, 0});
        full_grid[start_x][start_y] = 0;
        
        // Initialize reachable points
        std::unordered_map<std::string, std::set<std::pair<double, double>>> unique_points;
        
        // Check if start position is in any goal region
        double start_world_x = bounds[0] + start_x * resolution;
        double start_world_y = bounds[2] + start_y * resolution;
        
        
        // Complete BFS with goal checking
        while (!q.empty()) {
            auto [x, y, depth] = q.front();
            if (depth > radius) {
                break;
            }
            q.pop();
            
            for (const auto& [dx, dy] : dirs) {
                int nx = x + dx;
                int ny = y + dy;
                
                if (nx >= 0 && nx < grid_width && ny >= 0 && ny < grid_height && full_grid[nx][ny] == -1) {
                    double world_x = bounds[0] + nx * resolution;
                    double world_y = bounds[2] + ny * resolution;
                    full_grid[nx][ny] = 1;
                    q.push({nx, ny, depth + 1});
                }
            }
        }
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        // std::cout << "Wavefront computation took " << duration.count() << " milliseconds" << std::endl;
        return full_grid;
    }

    
    /**
     * @brief Check if a goal is reachable using the current grid
     */
    bool is_goal_reachable(
        const std::vector<double>& goal_pos,
        double goal_size = 0.05
    ) {
        // Calculate the grid bounds for the goal region
        int min_x = static_cast<int>(std::floor((goal_pos[0] - goal_size - bounds[0]) / resolution));
        int max_x = static_cast<int>(std::ceil((goal_pos[0] + goal_size - bounds[0]) / resolution));
        int min_y = static_cast<int>(std::floor((goal_pos[1] - goal_size - bounds[2]) / resolution));
        int max_y = static_cast<int>(std::ceil((goal_pos[1] + goal_size - bounds[2]) / resolution));
        
        // Clamp to grid bounds
        min_x = std::max(0, min_x);
        max_x = std::min(grid_width - 1, max_x);
        min_y = std::max(0, min_y);
        max_y = std::min(grid_height - 1, max_y);
        
        // Check if any cell in the goal region is reachable (value > 0)
        for (int x = min_x; x <= max_x; x++) {
            for (int y = min_y; y <= max_y; y++) {
                if (full_grid[x][y] >= 0) {
                    return true;
                }
            }
        }
        
        return false;
    }
    
    /**
     * @brief Save the current wavefront grid to a file
     */
    void save_wavefront_to_file(const std::string& filename) {
        std::ofstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("Could not open file: " + filename);
        }
        
        // Write grid data
        for (size_t x = 0; x < grid_width; x++) {
            for (size_t y = 0; y < grid_height; y++) {
                // Convert grid coordinates to world coordinates
                double world_x = bounds[0] + x * resolution;
                double world_y = bounds[2] + y * resolution;
                file << world_x << " " << world_y << " " << piecewise_grid[x][y] << "\n";
            }
        }
        file.close();
    }
    
    /**
     * @brief Get the current grid
     */
    const std::vector<std::vector<int>>& get_grid() const {
        return full_grid;
    }
};

// Keep the global functions for backward compatibility
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
        
        return std::abs(rotated[0] - obj_state->position[0]) <= obj.size[0] && 
               std::abs(rotated[1] - obj_state->position[1]) <= obj.size[1];
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
) {
    // Use the WavefrontPlanner class for more efficient implementation
    WavefrontPlanner planner(resolution, env, robot_size);
    return planner.compute_wavefront(env, start_pos, goal_positions);
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
 * @brief Checks if a goal region is reachable in the computed wavefront
 */
bool is_goal_reachable(
    const std::vector<std::vector<int>>& grid,
    const std::vector<double>& goal_pos,
    NAMOEnvironment& env,
    double resolution,
    double goal_size
) {
    // Calculate the grid bounds for the goal region
    auto bounds = env.get_environment_bounds();
    int min_x = static_cast<int>(std::floor((goal_pos[0] - goal_size - bounds[0]) / resolution));
    int max_x = static_cast<int>(std::ceil((goal_pos[0] + goal_size - bounds[0]) / resolution));
    int min_y = static_cast<int>(std::floor((goal_pos[1] - goal_size - bounds[2]) / resolution));
    int max_y = static_cast<int>(std::ceil((goal_pos[1] + goal_size - bounds[2]) / resolution));
    
    // Clamp to grid bounds
    min_x = std::max(0, min_x);
    max_x = std::min(static_cast<int>(grid.size()) - 1, max_x);
    min_y = std::max(0, min_y);
    max_y = std::min(static_cast<int>(grid[0].size()) - 1, max_y);
    
    // Check if any cell in the goal region is reachable (value > 0)
    for (int x = min_x; x <= max_x; x++) {
        for (int y = min_y; y <= max_y; y++) {
            if (grid[x][y] >= 0) {
                return true;
            }
        }
    }
    
    return false;
}

}  // namespace prx

#endif // WAVEFRONT_HPP