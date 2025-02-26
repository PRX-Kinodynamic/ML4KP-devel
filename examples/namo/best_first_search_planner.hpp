#pragma once

#include <queue>
#include <vector>
#include <unordered_map>
#include <cmath>
#include "motion_primitive_generator.hpp"


namespace prx {

struct SearchState {
    std::vector<double> state;  // x, y, theta
    double cost;
    int primitive_idx;
    int push_steps;
    SearchState* parent;

    SearchState(const std::vector<double>& state, double cost, int primitive_idx, int push_steps, SearchState* parent = nullptr)
        : state(state), cost(cost), primitive_idx(primitive_idx), push_steps(push_steps), parent(parent) {}
};

struct SearchStateCompare {
    bool operator()(const SearchState* a, const SearchState* b) {
        return a->cost > b->cost;  // Min heap
    }
};

struct PlanStep {
    int edge_idx;
    int push_steps;
    std::vector<double> pose;  // x, y, theta (SE(2) pose)
    
    PlanStep(int edge_idx, int push_steps, const std::vector<double>& pose)
        : edge_idx(edge_idx), push_steps(push_steps), pose(pose) {}
};

class GreedyBestFirstSearchPlanner {
public:
    static std::vector<PlanStep> plan_push_sequence(
        const std::vector<double>& start_state,
        const std::vector<double>& goal_state,
        const std::vector<MotionPrimitive>& primitives,
        const std::vector<int>& allowed_primitive_indices,
        double distance_threshold = 0.2,
        double angle_threshold = 0.2
    ) {

        // clean up search_states folder
        std::string search_states_folder = "search_states";
        if (std::filesystem::exists(search_states_folder)) {
            std::filesystem::remove_all(search_states_folder);
        }
        std::filesystem::create_directory(search_states_folder);

        // Get object info from first primitive (they should all be the same)
        const ObjectInfo& object_info = primitives[0].object_info;

        // Transform goal state relative to start state
        std::vector<double> transformed_goal = transform_to_local_frame(start_state, goal_state);
        
        // Initialize search
        std::priority_queue<SearchState*, std::vector<SearchState*>, SearchStateCompare> open_set;
        std::vector<SearchState*> all_states;  // For memory management
        
        // Start from origin (transformed start state)
        SearchState* start = new SearchState({0, 0, 0}, heuristic({0, 0, 0}, transformed_goal, object_info), -1, -1);
        open_set.push(start);
        all_states.push_back(start);

        int iter = 0;

        while (!open_set.empty()) {
            SearchState* current = open_set.top();
            open_set.pop();

            // Check if we reached the goal
            if (is_goal_reached(current->state, transformed_goal, object_info, distance_threshold, angle_threshold)) {
                std::vector<PlanStep> plan_sequence;
                std::vector<SearchState*> path;
                
                // Collect states in reverse order
                SearchState* trace = current;
                while (trace->parent != nullptr) {
                    path.push_back(trace);
                    std::cout << "edge_idx: " << trace->primitive_idx << " push_steps: " << trace->push_steps << std::endl;
                    trace = trace->parent;
                }
                
                // Reverse to get correct order
                std::reverse(path.begin(), path.end());
                
                // Transform local coordinates back to global
                for (auto state : path) {
                    // Find the primitive that was used
                    
                    // Get the global pose by transforming from local frame
                    std::vector<double> global_pose = transform_to_global_frame(
                        start_state, 
                        state->state
                    );
                    
                    // Create plan step with edge_idx, push_steps, and pose
                    plan_sequence.emplace_back(
                        state->primitive_idx,
                        state->push_steps,
                        global_pose
                    );
                }
                
                std::ofstream outfile("search_states/search_states_final.txt");
                outfile << std::setprecision(6);  // Set precision for floating point
                outfile << " # goal: " << transformed_goal[0] << " " << transformed_goal[1] << " " << transformed_goal[2] << "\n";
                for (auto state : all_states) {
                    outfile << state->state[0] << " " << state->state[1] << " " << state->state[2] << "\n";
                }
                outfile.close();

                // Cleanup
                for (auto state : all_states) {
                    delete state;
                }
                
                return plan_sequence;
            }

            // Expand current state using allowed primitives
            for (int idx = 0; idx < primitives.size(); idx++) {
                auto& primitive = primitives[idx];

                if (primitive.push_steps == 0) {
                    continue;
                }
                
                bool is_allowed = false;
                for (int i = 0; i < allowed_primitive_indices.size(); i++) {
                    if (allowed_primitive_indices[i] == primitive.edge_idx) {
                        is_allowed = true;
                        break;
                    }
                }
                if (is_allowed) {
                    // Apply primitive to get new state
                    std::vector<double> new_state = apply_primitive(current->state, primitive);
                    double new_cost = heuristic(new_state, transformed_goal, object_info);
                    SearchState* next_state = new SearchState(new_state, new_cost, primitive.edge_idx, primitive.push_steps, current);
                    open_set.push(next_state);
                    all_states.push_back(next_state);
                }
            }

            if (iter % 2 == 0) {


                std::ofstream outfile("search_states/search_states_" + std::to_string(iter) + ".txt");
                outfile << std::setprecision(6);  // Set precision for floating point
                // put goal state on top of the file
                outfile << " # goal: " << transformed_goal[0] << " " << transformed_goal[1] << " " << transformed_goal[2] << "\n";
                for (auto state : all_states) {
                    outfile << state->state[0] << " " << state->state[1] << " " << state->state[2] << "\n";
                }
                outfile.close();
            }
            iter++;

            if (iter > 500) {
                break;
            }
        }

        

        // Cleanup if no path found
        for (auto state : all_states) {
            delete state;
        }
        
        return {};  // Return empty sequence if no path found
    }

private:
    static std::vector<double> transform_to_local_frame(
        const std::vector<double>& reference,
        const std::vector<double>& target
    ) {
        double dx = target[0] - reference[0];
        double dy = target[1] - reference[1];
        double dtheta = target[2] - reference[2];
        
        // Rotate point by -reference[2]
        double cos_ref = std::cos(-reference[2]);
        double sin_ref = std::sin(-reference[2]);
        
        return {
            dx * cos_ref - dy * sin_ref,
            dx * sin_ref + dy * cos_ref,
            dtheta
        };
    }

    static double quaternion_distance_symmetric(
        const std::array<double, 4>& q1,
        const std::array<double, 4>& q2,
        int symmetry_rotations,
        bool scalar_first = true
    ) {
        // Normalize quaternions and handle scalar position
        auto normalize = [scalar_first](std::array<double, 4> q) {
            double norm = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
            std::array<double, 4> normalized = {q[0]/norm, q[1]/norm, q[2]/norm, q[3]/norm};
            
            // If scalar-last, convert to scalar-first for internal calculations
            if (!scalar_first) {
                normalized = {normalized[3], normalized[0], normalized[1], normalized[2]};
            }
            return normalized;
        };
        
        auto q1_norm = normalize(q1);
        auto q2_norm = normalize(q2);
        
        // Check all symmetric rotations
        double min_dist = std::numeric_limits<double>::infinity();
        for (int i = 0; i < symmetry_rotations; ++i) {
            double angle = i * (2 * M_PI / symmetry_rotations);
            
            // Create rotation quaternion around Z axis (in scalar-first format)
            double half_angle = angle * 0.5;
            std::array<double, 4> sym_rot = {
                std::cos(half_angle),  // w
                0,                     // x
                0,                     // y
                std::sin(half_angle)   // z
            };
            
            // Apply symmetric rotation (quaternion multiplication)
            std::array<double, 4> q2_sym = {
                sym_rot[0]*q2_norm[0] - sym_rot[3]*q2_norm[3],  // w
                sym_rot[0]*q2_norm[1] - sym_rot[3]*q2_norm[2],  // x
                sym_rot[0]*q2_norm[2] + sym_rot[3]*q2_norm[1],  // y
                sym_rot[0]*q2_norm[3] + sym_rot[3]*q2_norm[0]   // z
            };
            
            // Calculate distance
            double dot_product = std::abs(
                q1_norm[0]*q2_sym[0] + 
                q1_norm[1]*q2_sym[1] + 
                q1_norm[2]*q2_sym[2] + 
                q1_norm[3]*q2_sym[3]
            );
            dot_product = std::min(1.0, std::max(-1.0, dot_product));
            double dist = 1.0 - dot_product;
            
            min_dist = std::min(min_dist, dist);
        }
        
        return min_dist;
    }

    static double heuristic(
        const std::vector<double>& state,
        const std::vector<double>& goal,
        const ObjectInfo& object_info
    ) {
        // Position distance
        double dx = state[0] - goal[0];
        double dy = state[1] - goal[1];
        double pos_dist = std::sqrt(dx*dx + dy*dy);
        
        
        
        // Orientation distance with symmetry
        std::array<double, 4> q1 = yaw_to_quaternion(state[2], true);
        std::array<double, 4> q2 = yaw_to_quaternion(goal[2], true);
        double rot_dist = quaternion_distance_symmetric(q1, q2, object_info.symmetry_rotations, true);
        
        return pos_dist + 0.5 * rot_dist;
    }

    static std::vector<double> apply_primitive(
        const std::vector<double>& state,
        const MotionPrimitive& primitive
    ) {
        // Transform primitive effect to current state frame
        double cos_theta = std::cos(state[2]);
        double sin_theta = std::sin(state[2]);
        
        double dx = primitive.position[0];
        double dy = primitive.position[1];
        
        double yaw = quaternion_to_yaw(primitive.quaternion, true);


        
        return {
            state[0] + dx * cos_theta - dy * sin_theta,
            state[1] + dx * sin_theta + dy * cos_theta,
            state[2] + yaw
        };
    }

    static bool is_goal_reached(
        const std::vector<double>& state,
        const std::vector<double>& goal,
        const ObjectInfo& object_info,
        double distance_threshold,
        double angle_threshold
    ) {
        // Position check
        double dx = state[0] - goal[0];
        double dy = state[1] - goal[1];
        double distance = std::sqrt(dx*dx + dy*dy);
        
        // Orientation check with symmetry
        std::array<double, 4> q1 = yaw_to_quaternion(state[2], true);
        std::array<double, 4> q2 = yaw_to_quaternion(goal[2], true);
        double rot_dist = quaternion_distance_symmetric(q1, q2, object_info.symmetry_rotations, true);
        
        return distance < distance_threshold && rot_dist < angle_threshold;
    }

    static std::vector<double> transform_to_global_frame(
        const std::vector<double>& reference,
        const std::vector<double>& local
    ) {
        double cos_ref = std::cos(reference[2]);
        double sin_ref = std::sin(reference[2]);
        
        // Rotate point by reference[2]
        double x = local[0] * cos_ref - local[1] * sin_ref;
        double y = local[0] * sin_ref + local[1] * cos_ref;
        
        return {
            reference[0] + x,
            reference[1] + y,
            reference[2] + local[2]
        };
    }
};

} // namespace prx 