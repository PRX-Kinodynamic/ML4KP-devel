#pragma once
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <algorithm>
#include <utility>
#include <memory>
#include <functional>
#include <queue>
#include "wavefront_planner.hpp"
#include "environment.hpp"

namespace prx {

/**
 * @brief Custom hash function for std::pair<int, int>
 */
struct PairHash {
    size_t operator()(const std::pair<int, int>& p) const {
        size_t seed = std::hash<int>()(p.first);
        // Better mixing with prime numbers and bit rotation
        seed ^= std::hash<int>()(p.second) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }
};

/**
 * @brief Information about a region in the graph
 */
struct RegionInfo {
    std::vector<std::pair<int, int>> cells;      // All cells in the region
    std::unordered_set<int> neighbors;           // Adjacent regions
    std::unordered_set<std::pair<int, int>, PairHash> boundary_cells; // Boundary cells
};

/**
 * @brief A graph representation of connected regions in the environment
 * 
 * Maintains a dynamic graph where:
 * - Nodes represent distinct regions identified by wavefront exploration
 * - Edges represent adjacency between regions
 * - Edges store information about objects separating the regions
 */
class DynamicRegionGraph {
public:
    /**
     * @brief Constructor that builds an initial graph
     * 
     * @param planner Reference to the wavefront planner
     * @param env Reference to the environment
     */
    DynamicRegionGraph(WavefrontPlanner& planner, NAMOEnvironment& env)
        : planner(planner), env(env) 
    {
        // Precompute likely sizes based on environment
        const size_t grid_size = planner.get_grid_width() * planner.get_grid_height();
        const size_t estimated_regions = grid_size / 100;  // Assuming avg region size is 10x10
        const size_t max_movable_objects = env.get_movable_objects().size();
        const size_t estimated_edges = estimated_regions * 4;  // Average connectivity
        
        // Pre-allocate all major containers
        regions.reserve(estimated_regions);
        separating_objects.reserve(estimated_edges);
        
        // Pre-allocate reusable working containers
        m_affected_edges.reserve(8);
        m_region_vec.reserve(8);
        m_neighbor_set.reserve(8);
        m_object_set.reserve(max_movable_objects);
        m_explored_starts.reserve(grid_size / 20);  // Assume we explore ~5% of grid
        m_boundaries.reserve(grid_size / 10);       // Assume ~10% of cells are boundaries
        
        // Initialize visited array once (reused in build_initial_graph)
        m_visited.resize(planner.get_grid_width(), std::vector<bool>(planner.get_grid_height(), false));
        
        // Initialize the graph
        build_initial_graph();
        print_graph();
    }
    
    /**
     * @brief Get adjacency list for a region
     * 
     * @param region_id The region ID
     * @return Vector of adjacent region IDs
     */
    const std::unordered_set<int>& get_adjacent_regions(int region_id) const {
        static const std::unordered_set<int> empty_set;
        auto it = regions.find(region_id);
        if (it != regions.end()) {
            return it->second.neighbors;
        }
        return empty_set;
    }
    
    /**
     * @brief Get objects separating two regions
     * 
     * @param region1 First region ID
     * @param region2 Second region ID
     * @return Vector of object names
     */
    std::vector<std::string> get_separating_objects(int region1, int region2) const {
        std::pair<int, int> edge(std::min(region1, region2), std::max(region1, region2));
        auto it = separating_objects.find(edge);
        if (it != separating_objects.end()) {
            return it->second;
        }
        return {};
    }
    
    /**
     * @brief Get all region IDs in the graph
     * 
     * @return Set of region IDs
     */
    std::unordered_set<int> get_all_regions() const {
        std::unordered_set<int> region_ids;
        region_ids.reserve(regions.size());
        for (const auto& [region, _] : regions) {
            region_ids.insert(region);
        }
        return region_ids;
    }
    
    /**
     * @brief Get the region ID at a specific point
     * 
     * @param x X coordinate in world space
     * @param y Y coordinate in world space
     * @return Region ID or -1 if not in any region
     */
    int get_region_at_point(double x, double y) const {
        // Convert to grid coordinates
        int grid_x = static_cast<int>((x - planner.get_bounds()[0]) / planner.get_resolution());
        int grid_y = static_cast<int>((y - planner.get_bounds()[2]) / planner.get_resolution());
        
        // Check bounds
        if (grid_x < 0 || grid_x >= planner.get_grid_width() || 
            grid_y < 0 || grid_y >= planner.get_grid_height()) {
            return -1;
        }
        
        // Return region ID from grid
        return planner.get_piecewise_grid()[grid_x][grid_y];
    }

    void mark_point_as(double x, double y, int mark=100) {
        // Convert to grid coordinates
        int grid_x = static_cast<int>((x - planner.get_bounds()[0]) / planner.get_resolution());
        int grid_y = static_cast<int>((y - planner.get_bounds()[2]) / planner.get_resolution());
        
        // Check bounds
        if (grid_x < 0 || grid_x >= planner.get_grid_width() || 
            grid_y < 0 || grid_y >= planner.get_grid_height()) {
        }
        planner.get_mutable_piecewise_grid()[grid_x][grid_y] = mark;
        // Return region ID from grid
        // return planner.get_piecewise_grid()[grid_x][grid_y];
    }
    
    /**
     * @brief Update graph after an object has been moved
     * 
     * This updates the graph structure to reflect changes in connectivity
     * after an object has been moved in the environment.
     * 
     * @param object_name Name of the moved object
     */
    void update_after_object_moved(const std::string& object_name) {
        // 1. Identify edges affected by this object
        m_affected_edges.clear();
        
        auto map_it = separating_objects.begin();
        while (map_it != separating_objects.end()) {
            auto vector_it = std::find(map_it->second.begin(), map_it->second.end(), object_name);
            if (vector_it != map_it->second.end()) {
                m_affected_edges.push_back(map_it->first);
                map_it->second.erase(vector_it);
                
                // Remove entry immediately if empty
                if (map_it->second.empty()) {
                    map_it = separating_objects.erase(map_it);
                    continue;
                }
            }
            ++map_it;
        }
        
        // 2. For each affected edge, check if regions should be merged
        for (const auto& edge : m_affected_edges) {
            int region1 = edge.first;
            int region2 = edge.second;
            
            // If edge exists and no objects left separating these regions, merge them
            auto it = separating_objects.find(edge);
            if (it != separating_objects.end() && it->second.empty()) {
                merge_regions(region1, region2);
            }
        }
        
        // IMPORTANT: Add this line to reset the piecewise grid based on current object positions
        // planner.reset_piecewise_grid(env);
        
        // 3. Explore new accessible areas
        const auto* obj_state = env.get_object_state(object_name);
        if (obj_state) {
            std::vector<double> obj_pos = {obj_state->position[0], obj_state->position[1]};
            
            // Find nearby unexplored areas around the object
            explore_near_point(obj_pos, 5);
        }
        
        // 4. Update graph with new region information
        update_region_connectivity();
        // print_graph();
    }
    
    /**
     * @brief Rebuild the entire graph from scratch
     * 
     * Use this when the environment has changed significantly
     */
    void rebuild_graph() {
        // Clear existing graph data
        regions.clear();
        separating_objects.clear();
        
        // Reset the piecewise grid
        planner.reset_piecewise_grid(env);
        
        // Reset the next region ID
        next_region_id = 1;
        
        // Rebuild from scratch
        build_initial_graph();
    }
    
    /**
     * @brief Find the shortest path between two regions
     * 
     * @param start_region Starting region ID
     * @param goal_region Goal region ID
     * @return Vector of region IDs forming the path
     */
    std::vector<int> find_path_between_regions(int start_region, int goal_region) {
        if (start_region == goal_region) {
            return {start_region};
        }

        // std::cout << "start_region: " << start_region << std::endl;
        // std::cout << "goal_region: " << goal_region << std::endl;
        
        // Reuse member containers for BFS
        m_queue = std::queue<int>(); // Clear the queue
        m_parent.clear();
        m_visited_regions.clear();
        
        // Push start region
        m_queue.push(start_region);
        m_visited_regions.insert(start_region);
        
        while (!m_queue.empty()) {
            int current = m_queue.front();
            m_queue.pop();
            
            if (current == goal_region) {
                // Reconstruct path
                std::vector<int> path;
                path.reserve(m_visited_regions.size()); // Maximum possible path length
                
                int node = current;
                while (node != start_region) {
                    path.push_back(node);
                    node = m_parent[node];
                }
                path.push_back(start_region);
                std::reverse(path.begin(), path.end());
                return path;
            }
            
            // Add defensive check for region validity
            auto it = regions.find(current);
            if (it == regions.end()) continue;
            
            // Use the found iterator instead of .at() which can throw
            const auto& neighbors = it->second.neighbors;
            // how can i randomize the neighbors?
            std::vector<int> neighbors_vec(neighbors.begin(), neighbors.end());
            std::random_device rd;
            std::mt19937 g(rd());
            std::shuffle(neighbors_vec.begin(), neighbors_vec.end(), g);
            for (int neighbor : neighbors_vec) {
                if (m_visited_regions.count(neighbor) == 0) {
                    m_visited_regions.insert(neighbor);
                    m_parent[neighbor] = current;
                    m_queue.push(neighbor);
                }
            }
        }

        // No path found
        std::cout << "No path found" << std::endl;
        return {};
    }
    
    /**
     * @brief Find objects to move to connect two regions
     * 
     * @param start_region Starting region ID
     * @param goal_region Goal region ID
     * @return Vector of object names to move
     */
        // Find path between regions
    std::vector<std::vector<std::string>> find_objects_to_move(int start_region, int goal_region) {
        std::vector<int> region_path = find_path_between_regions(start_region, goal_region);
        
        // If no path, return empty list
        if (region_path.empty() || region_path.size() == 1) {
            return {};
        }

        for (const auto& region : region_path){
            std::cout << "region: " << region << std::endl;
        }
        
        // Collect objects along the path
        std::vector<std::vector<std::string>> objects_by_edge;
        
        for (size_t i = 0; i < region_path.size() - 1; i++) {
            int r1 = region_path[i];
            int r2 = region_path[i + 1];
            
            std::pair<int, int> edge(std::min(r1, r2), std::max(r1, r2));
            std::cout << "edge: " << edge.first << " " << edge.second << std::endl;
            // Use find instead of operator[] to avoid creating empty entries
            auto it = separating_objects.find(edge);
            if (it != separating_objects.end()) {
                // Add all objects to the set (avoids duplicates)
                for (const auto& obj : it->second){
                    std::cout << "object: " << obj << std::endl;
                }
                objects_by_edge.push_back(it->second);
            }
        }
        
        return objects_by_edge;
    }


    bool is_connected_to_goal(int region_id) {
        auto it = regions.find(region_id);
        // if (it == regions.end()) return false;
        const auto& neighbors = it->second.neighbors;

        for (int neighbor : neighbors) {
            std::cout << "neighbor: " << neighbor << std::endl;
            if (neighbor == 0) return true;
        }
        return false;
    }
    
    /**
     * @brief Get the total number of regions in the graph
     */
    size_t get_region_count() const {
        return regions.size();
    }
    
    /**
     * @brief Print the graph structure for debugging
     */
    void print_graph() const {
        std::cout << "Region Graph Summary:" << std::endl;
        std::cout << "Total regions: " << regions.size() << std::endl;
        
        for (const auto& [region, info] : regions) {
            std::cout << "Region " << region << " connections:" << std::endl;
            
            for (int neighbor : info.neighbors) {
                std::pair<int, int> edge(std::min(region, neighbor), std::max(region, neighbor));
                std::cout << "  → Connected to region " << neighbor << ", separated by: ";
                for (const auto& obj : separating_objects.at(edge)) {
                    std::cout << obj << " ";
                }
                std::cout << std::endl;
            }
            
            std::cout << "  This region has " << info.cells.size() << " cells and " 
                      << info.boundary_cells.size() << " boundary cells" << std::endl;
        }
    }
    
    /**
     * @brief Find the nearest valid region to a point
     * 
     * @param x X coordinate in world space
     * @param y Y coordinate in world space
     * @param search_radius The approximate radius (in grid cells) to search
     * @return Region ID or -1 if no valid region found nearby
     */
    int find_nearest_valid_region(double x, double y, int search_radius = 3) {
        // Convert to grid coordinates
        int grid_x = static_cast<int>((x - planner.get_bounds()[0]) / planner.get_resolution());
        int grid_y = static_cast<int>((y - planner.get_bounds()[2]) / planner.get_resolution());
        
        // First check if current position is valid
        int current_region = get_region_at_point(x, y);
        std::cout << "current_region: " << current_region << std::endl;
        if (current_region > 0) return current_region;
        
        // Search in expanding rings
        for (int r = 1; r <= search_radius; r++) {
            for (int dx = -r; dx <= r; dx++) {
                for (int dy = -r; dy <= r; dy++) {
                    // Only check points at distance r (the perimeter)
                    if (std::abs(dx) == r || std::abs(dy) == r) {
                        int nx = grid_x + dx;
                        int ny = grid_y + dy;
                        
                        // Check bounds
                        if (nx < 0 || nx >= planner.get_grid_width() || 
                            ny < 0 || ny >= planner.get_grid_height()) {
                            continue;
                        }
                        
                        // Check if this is a valid region
                        if (planner.get_piecewise_grid()[nx][ny] > 0) {
                            return planner.get_piecewise_grid()[nx][ny];
                        }
                    }
                }
            }
        }
        
        // No valid region found nearby
        return -1;
    }
    
private:
    WavefrontPlanner& planner;
    NAMOEnvironment& env;
    
    // Graph representation
    std::unordered_map<int, RegionInfo> regions;
    std::unordered_map<std::pair<int, int>, std::vector<std::string>, PairHash> separating_objects;
    
    // Next region ID for exploration
    int next_region_id = 1;
    
    // 8-connected neighborhood directions (cached)
    const std::vector<std::pair<int, int>> dirs = {
        {1,0}, {-1,0}, {0,1}, {0,-1}, {1,1}, {1,-1}, {-1,1}, {-1,-1}
    };
    
    // Pre-allocated reusable containers to avoid runtime allocations
    std::vector<std::pair<int, int>> m_affected_edges;
    std::unordered_set<std::pair<int, int>, PairHash> m_explored_starts;
    std::unordered_set<std::pair<int, int>, PairHash> m_boundaries;
    std::unordered_set<int> m_region_ids;
    std::vector<int> m_region_vec;
    std::unordered_set<int> m_neighbor_set;
    std::unordered_set<std::string> m_object_set;
    std::vector<std::vector<bool>> m_visited;
    
    // BFS data structures (reused across calls)
    std::queue<int> m_queue;
    std::unordered_map<int, int> m_parent;
    std::unordered_set<int> m_visited_regions;
    
    /**
     * @brief Build the initial graph from the environment
     */
    void build_initial_graph() {
        // Reset the piecewise grid first
        planner.reset_piecewise_grid(env);


        // initializes the regions of the graph using region_info struct
        
        // Clear visited array
        for (auto& row : m_visited) {
            std::fill(row.begin(), row.end(), false);
        }

        
        RegionInfo& region_info = regions[0];
        region_info.cells.reserve(planner.get_grid_width() * planner.get_grid_height() / 100);
        region_info.neighbors.reserve(8);
        region_info.boundary_cells.reserve(planner.get_grid_width() * planner.get_grid_height() / 50);
        std::array<double, 2> robot_goal = env.get_robot_goal();
        int goal_region = find_nearest_valid_region(robot_goal[0], robot_goal[1]);
        std::cout << "goal_region: " << goal_region << std::endl;
        for (int x = 0; x < planner.get_grid_width(); x++) {
            for (int y = 0; y < planner.get_grid_height(); y++) {
                if (planner.get_piecewise_grid()[x][y] == 0) {
                    for (int i = 0; i < planner.get_grid_width(); i++) {
                            for (int j = 0; j < planner.get_grid_height(); j++) {
                                if (planner.get_piecewise_grid()[i][j] == 0) {
                                    m_visited[i][j] = true;
                                    // Add to list of cells in this region
                                    region_info.cells.emplace_back(i, j);
                                    
                                    // Check if this is a boundary cell
                                    bool is_boundary = false;
                                    for (const auto& [dx, dy] : dirs) {
                                        int nx = i + dx;
                                        int ny = j + dy;
                                        
                                        if (nx >= 0 && nx < planner.get_grid_width() && 
                                            ny >= 0 && ny < planner.get_grid_height() && 
                                            (planner.get_piecewise_grid()[nx][ny] == -2 || planner.get_piecewise_grid()[nx][ny] == -1)) {
                                            is_boundary = true;
                                            break;
                                        }
                                    }
                                    
                                    if (is_boundary) {
                                        region_info.boundary_cells.emplace(std::make_pair(i, j));
                                    }
                                }
                            }
                        }
                }
            }
        }
        
        for (int x = 0; x < planner.get_grid_width(); x++) {
            for (int y = 0; y < planner.get_grid_height(); y++) {
                // Skip obstacle cells and already explored cells
                if (planner.get_piecewise_grid()[x][y] == -2 ||  m_visited[x][y]) {
                    continue;
                }
                
                if (planner.get_piecewise_grid()[x][y] == -1) {
                    // Found unexplored free cell - start a new region
                    std::vector<double> start_pos = {
                        planner.get_bounds()[0] + x * planner.get_resolution(),
                        planner.get_bounds()[2] + y * planner.get_resolution()
                    };
                    
                    // Compute wavefront from this point
                    auto [grid, region_data, distances, success] = 
                        planner.compute_piecewise_wavefront(env, start_pos, {}, next_region_id);
                    
                    if (success) {
                        // Initialize a new region
                        RegionInfo& region_info = regions[next_region_id];
                        
                        // Reserve space based on grid size
                        region_info.cells.reserve(planner.get_grid_width() * planner.get_grid_height() / 100);
                        region_info.neighbors.reserve(8);
                        region_info.boundary_cells.reserve(planner.get_grid_width() * planner.get_grid_height() / 50);
                        
                        // Mark explored cells as visited and collect cell info
                        for (int i = 0; i < planner.get_grid_width(); i++) {
                            for (int j = 0; j < planner.get_grid_height(); j++) {
                                if (planner.get_piecewise_grid()[i][j] == next_region_id) {
                                    m_visited[i][j] = true;
                                    
                                    // Add to list of cells in this region
                                    region_info.cells.emplace_back(i, j);
                                    
                                    // Check if this is a boundary cell
                                    bool is_boundary = false;
                                    for (const auto& [dx, dy] : dirs) {
                                        int nx = i + dx;
                                        int ny = j + dy;
                                        
                                        if (nx >= 0 && nx < planner.get_grid_width() && 
                                            ny >= 0 && ny < planner.get_grid_height() && 
                                            planner.get_piecewise_grid()[nx][ny] == -2 || planner.get_piecewise_grid()[nx][ny] == 0) {
                                            is_boundary = true;
                                            break;
                                        }
                                    }
                                    
                                    if (is_boundary) {
                                        region_info.boundary_cells.emplace(std::make_pair(i, j));
                                    }
                                }
                            }
                        }
                        
                        next_region_id++;
                    }
                }
            }
        }
        
        // Build the graph connectivity
        build_region_connectivity();
    }
    
    /**
     * @brief Build region connectivity by analyzing the grid
     */
    void build_region_connectivity() {
        // Find all unique region IDs
        m_region_ids.clear();
        for (int x = 0; x < planner.get_grid_width(); x++) {
            for (int y = 0; y < planner.get_grid_height(); y++) {
                if (planner.get_piecewise_grid()[x][y] >= 0) {
                    m_region_ids.insert(planner.get_piecewise_grid()[x][y]);
                }
            }
        }
        
        // Make sure all regions exist and clear their neighbor lists
        for (int id : m_region_ids) {
            if (regions.find(id) == regions.end()) {
                // Initialize a new region
                regions[id].neighbors.reserve(8);
            } else {
                // Clear existing neighbors
                regions[id].neighbors.clear();
            }
        }
        
        // Static map for object positions - reused across calls
        static std::unordered_map<std::pair<int, int>, std::string, PairHash> obstacle_objects;
        obstacle_objects.clear();
        obstacle_objects.reserve(env.get_movable_objects().size() * 100);  // Estimate cells per object
        
        // Pre-compute object positions
        for (int x = 0; x < planner.get_grid_width(); x++) {
            for (int y = 0; y < planner.get_grid_height(); y++) {
                if (planner.get_piecewise_grid()[x][y] == 0) {
                    obstacle_objects[std::make_pair(x, y)] = "goal";
                }

                if (planner.get_piecewise_grid()[x][y] == -2) {
                    
                    double world_x = planner.get_bounds()[0] + x * planner.get_resolution();
                    double world_y = planner.get_bounds()[2] + y * planner.get_resolution();
                
                    for (const auto& obj : env.get_movable_objects()) {
                        NAMOEnvironment::ObjectInfo inflated_obj = obj;
                        const NAMOEnvironment::ObjectState* obj_state = env.get_object_state(obj.name);
                        
                        inflated_obj.size[0] += planner.get_robot_size()[0];
                        inflated_obj.size[1] += planner.get_robot_size()[0];
                        
                        if (is_point_in_rotated_object(world_x, world_y, inflated_obj, obj_state)) {
                            obstacle_objects[std::make_pair(x, y)] = obj.name;
                            // std::cout << "DEBUG: Cell (" << x << ", " << y << ") contains object " << obj.name << std::endl;
                            break;
                        }
                    }
                }
            }
        }
        
        // Create a map of objects to all regions they're adjacent to
        std::unordered_map<std::string, std::unordered_set<int>> object_to_regions;

        // Collect all regions adjacent to each object
        for (const auto& [pos, obj_name] : obstacle_objects) {

            // if (obj_name == "goal") continue;
            int x = pos.first;
            int y = pos.second;
            
            for (const auto& [dx, dy] : dirs) {
                int nx = x + dx;
                int ny = y + dy;
                if (nx >= 0 && nx < planner.get_grid_width() && 
                    ny >= 0 && ny < planner.get_grid_height()){
                        if (planner.get_piecewise_grid()[nx][ny] >= 0) {
                            if (planner.get_piecewise_grid()[x][y] == 0 && planner.get_piecewise_grid()[nx][ny] == 0) {
                                continue;
                            }
                            object_to_regions[obj_name].insert(planner.get_piecewise_grid()[nx][ny]);
                        }
                    }
            }
        }
    

        // Connect regions that share objects
        for (const auto& [obj_name, adjacent_regions] : object_to_regions) {

            if (obj_name == "goal") {
                if (adjacent_regions.size() > 0) {
                    std::vector<int> region_vec(adjacent_regions.begin(), adjacent_regions.end());
                    for (size_t i = 0; i < region_vec.size(); ++i) {
                            int region1 = 0;
                            int region2 = region_vec[i];
                            regions[region1].neighbors.insert(region2);
                            regions[region2].neighbors.insert(region1);

                            std::pair<int, int> edge(0, region2);
                            if (std::find(separating_objects[edge].begin(), separating_objects[edge].end(), obj_name) == separating_objects[edge].end()) {
                                separating_objects[edge].push_back(obj_name);
                            }
                    }
                }
            }

            else if (adjacent_regions.size() >= 2) {
                // std::cout << "DEBUG: Object " << obj_name << " connects regions: ";
                // for (int reg : adjacent_regions) {
                //     std::cout << reg << " ";
                // }
                // std::cout << std::endl;
                
                // Convert set to vector for pairwise iteration
                std::vector<int> region_vec(adjacent_regions.begin(), adjacent_regions.end());
                
                // Create edges between all pairs
                for (size_t i = 0; i < region_vec.size(); ++i) {
                    for (size_t j = i + 1; j < region_vec.size(); ++j) {
                        int region1 = region_vec[i];
                        int region2 = region_vec[j];
                        
                        // Create edge
                        regions[region1].neighbors.insert(region2);
                        regions[region2].neighbors.insert(region1);
                        
                        // Add object to separating objects
                        std::pair<int, int> edge(std::min(region1, region2), std::max(region1, region2));
                        if (std::find(separating_objects[edge].begin(), separating_objects[edge].end(), obj_name) == separating_objects[edge].end()) {
                            separating_objects[edge].push_back(obj_name);
                        }
                        
                        // std::cout << "DEBUG: Added edge between regions " << region1 << " and " << region2 
                                //   << " separated by " << obj_name << std::endl;
                    }
                }
            }
        }
        // Add after line ~538 to check how many obstacle cells have objects
        // std::cout << "DEBUG: Found " << obstacle_objects.size() << " cells occupied by movable objects" << std::endl;
    }
    
    /**
     * @brief Merge two regions into one
     * 
     * @param region1 First region ID
     * @param region2 Second region ID
     */
    void merge_regions(int region1, int region2) {
        // Use the smaller region ID as the target
        int target = std::min(region1, region2);
        int source = std::max(region1, region2);
        
        // Only update the cells we know are in the source region
        for (const auto& [x, y] : regions[source].cells) {
            std::cout << "before " << target << std::endl;
            std::cout << planner.get_piecewise_grid()[x][y] << std::endl;
            planner.get_mutable_piecewise_grid()[x][y] = target;
            std::cout << "after" << std::endl;
            std::cout << planner.get_piecewise_grid()[x][y] << std::endl;
        }
        
        // Get references to the region info
        RegionInfo& target_info = regions[target];
        RegionInfo& source_info = regions[source];
        
        // Add source cells to target region
        target_info.cells.insert(
            target_info.cells.end(),
            source_info.cells.begin(),
            source_info.cells.end()
        );
        
        // Add source boundary cells to target
        target_info.boundary_cells.insert(
            source_info.boundary_cells.begin(),
            source_info.boundary_cells.end()
        );
        
        // Create lookup set for target neighbors for faster checks
        m_neighbor_set.clear();
        m_neighbor_set.insert(target_info.neighbors.begin(), target_info.neighbors.end());
        
        // Add all of source's neighbors to target
        for (int neighbor : source_info.neighbors) {
            if (neighbor != target) {  // Skip self-loops
                // Add to target's adjacency list if not already there
                if (m_neighbor_set.count(neighbor) == 0) {
                    target_info.neighbors.insert(neighbor);
                    m_neighbor_set.insert(neighbor);
                }
                
                // Update neighbor's adjacency list
                auto& neighbor_info = regions[neighbor];
                neighbor_info.neighbors.erase(source);
                neighbor_info.neighbors.insert(target);
                
                // Update edge data
                std::pair<int, int> source_edge(std::min(source, neighbor), std::max(source, neighbor));
                std::pair<int, int> target_edge(std::min(target, neighbor), std::max(target, neighbor));
                
                // Get separating objects lists
                auto& source_objects = separating_objects[source_edge];
                auto& target_objects = separating_objects[target_edge];
                
                // Create lookup set for target objects
                m_object_set.clear();
                m_object_set.insert(target_objects.begin(), target_objects.end());
                
                // Add unique objects from source to target
                for (const auto& obj : source_objects) {
                    if (m_object_set.count(obj) == 0) {
                        target_objects.push_back(obj);
                        m_object_set.insert(obj);
                    }
                }
                
                // Remove old edge
                separating_objects.erase(source_edge);
            }
        }
        
        // Remove edge between the merged regions
        std::pair<int, int> merged_edge(std::min(region1, region2), std::max(region1, region2));
        separating_objects.erase(merged_edge);
        
        // Remove the merged region from the graph
        regions.erase(source);
    }
    
    /**
     * @brief Update region connectivity after changes
     */
    void update_region_connectivity() {
        // Clear existing neighbor relations but keep the regions
        for (auto& [region_id, info] : regions) {
            info.neighbors.clear();
        }
        separating_objects.clear();
        
        // Rebuild connectivity
        build_region_connectivity();
    }
    
    /**
     * @brief Explore the environment near a point
     * 
     * @param point The point to explore around
     * @param radius The approximate radius (in grid cells) to explore
     */
    void explore_near_point(const std::vector<double>& point, int radius) {
        // Convert to grid coordinates
        int grid_x = static_cast<int>((point[0] - planner.get_bounds()[0]) / planner.get_resolution());
        int grid_y = static_cast<int>((point[1] - planner.get_bounds()[2]) / planner.get_resolution());
        
        // Check bounds
        if (grid_x < 0 || grid_x >= planner.get_grid_width() || 
            grid_y < 0 || grid_y >= planner.get_grid_height()) {
            return;
        }
        
        // Clear reused container
        m_explored_starts.clear();
        
        // Spiral outward to find unexplored cells
        for (int r = 1; r <= radius; r++) {
            // For each cell at approximate distance r
            for (int dx = -r; dx <= r; dx++) {
                for (int dy = -r; dy <= r; dy++) {
                    // Only consider cells at approximate distance r
                    if (std::abs(dx) == r || std::abs(dy) == r) {
                        int nx = grid_x + dx;
                        int ny = grid_y + dy;
                        
                        // Skip if out of bounds
                        if (nx < 0 || nx >= planner.get_grid_width() || 
                            ny < 0 || ny >= planner.get_grid_height()) {
                            continue;
                        }
                        
                        // Skip if already explored or if not free
                        if (planner.get_piecewise_grid()[nx][ny] != -1) {
                            continue;
                        }
                        
                        // Skip if we've already tried exploring from this cell
                        std::pair<int, int> pos_pair(nx, ny);
                        if (m_explored_starts.count(pos_pair) > 0) {
                            continue;
                        }
                        
                        m_explored_starts.insert(pos_pair);
                        
                        // Prepare to explore from this point
                        std::vector<double> start_pos = {
                            planner.get_bounds()[0] + nx * planner.get_resolution(),
                            planner.get_bounds()[2] + ny * planner.get_resolution()
                        };
                        
                        // Explore from this point
                        auto [grid, region_data, distances, success] = 
                            planner.compute_piecewise_wavefront(env, start_pos, {}, next_region_id);
                        
                        if (success) {
                            // Initialize a new region
                            RegionInfo& region_info = regions[next_region_id];
                            
                            // Reserve space based on grid size
                            region_info.cells.reserve(planner.get_grid_width() * planner.get_grid_height() / 100);
                            region_info.boundary_cells.reserve(planner.get_grid_width() * planner.get_grid_height() / 50);
                            
                            for (int i = 0; i < planner.get_grid_width(); i++) {
                                for (int j = 0; j < planner.get_grid_height(); j++) {
                                    if (planner.get_piecewise_grid()[i][j] == next_region_id) {
                                        // Add to list of cells in this region
                                        region_info.cells.emplace_back(i, j);
                                        
                                        // Check if this is a boundary cell
                                        bool is_boundary = false;
                                        for (const auto& [d_x, d_y] : dirs) {
                                            int n_x = i + d_x;
                                            int n_y = j + d_y;
                                            
                                            if (n_x >= 0 && n_x < planner.get_grid_width() && 
                                                n_y >= 0 && n_y < planner.get_grid_height() && 
                                                planner.get_piecewise_grid()[n_x][n_y] == -2) {
                                                is_boundary = true;
                                                break;
                                            }
                                        }
                                        
                                        if (is_boundary) {
                                            region_info.boundary_cells.emplace(std::make_pair(i, j));
                                        }
                                    }
                                }
                            }
                            
                            next_region_id++;
                        }
                    }
                }
            }
        }
    }
};

} // namespace prx