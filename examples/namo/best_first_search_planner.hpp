#pragma once

#include <queue>
#include <vector>
#include <unordered_map>
#include <cmath>
#include "motion_primitive_generator.hpp"
#include "environment.hpp"
#include "namo_utility.hpp"

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
        const int& symmetry_rotations,
        int best_first_expansion_limit,
        std::function<double(const std::vector<double>&, const std::vector<double>&, const int)> get_distance = nullptr,
        std::function<bool(const std::vector<double>&, const std::vector<double>&, const int)> is_goal_reached_fn = nullptr
    ) {
        // These functions are now passed as parameters with default values of nullptr

        // clean up search_states folder
        std::string search_states_folder = "search_states";
        if (std::filesystem::exists(search_states_folder)) {
            std::filesystem::remove_all(search_states_folder);
        }
        std::filesystem::create_directory(search_states_folder);

        // Get object info from first primitive (they should all be the same)
        // Transform goal state relative to start state
        std::vector<double> transformed_goal = transform_to_local_frame(start_state, goal_state);
        
        // Initialize search
        std::priority_queue<SearchState*, std::vector<SearchState*>, SearchStateCompare> open_set;
        std::vector<SearchState*> all_states;  // For memory management
        
        // Start from origin (transformed start state)
        SearchState* start = new SearchState({0, 0, 0}, get_distance({0, 0, 0}, transformed_goal, symmetry_rotations), -1, -1);
        open_set.push(start);
        all_states.push_back(start);

        // Track best node (closest to goal)
        SearchState* best_node = start;
        double best_heuristic = best_node->cost;

        int iter = 0;

        while (!open_set.empty()) {
            SearchState* current = open_set.top();
            open_set.pop();

            // Check if we reached the goal
            if (is_goal_reached_fn(current->state, transformed_goal, symmetry_rotations)) {
                std::vector<PlanStep> plan_sequence;
                std::vector<SearchState*> path;
                
                // Collect states in reverse order
                SearchState* trace = current;
                while (trace != nullptr) {
                    if (trace->parent == nullptr) {
                        trace->push_steps = 0;
                    }
                    path.push_back(trace);
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
                
                // std::ofstream outfile("search_states/search_states_final.txt");
                // outfile << std::setprecision(6);  // Set precision for floating point
                // outfile << " # goal: " << transformed_goal[0] << " " << transformed_goal[1] << " " << transformed_goal[2] << "\n";
                // for (auto state : all_states) {
                //     outfile << state->state[0] << " " << state->state[1] << " " << state->state[2] << "\n";
                // }
                // outfile.close();

                // Cleanup
                for (auto state : all_states) {
                    delete state;
                }
                
                return plan_sequence;
            }

            // Update best node if this one is closer to the goal
            double current_heuristic = get_distance(current->state, transformed_goal, symmetry_rotations);
            if (current_heuristic < best_heuristic) {
                best_node = current;
                best_heuristic = current_heuristic;

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
                    double new_cost = get_distance(new_state, transformed_goal, symmetry_rotations);
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

            if (iter > best_first_expansion_limit) {
                break;
            }
        }

        // If we get here, we didn't find a path to goal
        // Instead of returning empty, return path to the best node
        
        std::vector<PlanStep> plan_sequence;
        std::vector<SearchState*> path;
        
        // Collect states in reverse order
        SearchState* trace = best_node;
        while (trace != nullptr) {
            if (trace->parent == nullptr) {
                trace->push_steps = 0;
            }
            path.push_back(trace);
            trace = trace->parent;
        }
        
        // Reverse to get correct order
        std::reverse(path.begin(), path.end());
        
        // Transform local coordinates back to global
        for (auto state : path) {
            std::vector<double> global_pose = transform_to_global_frame(
                start_state, 
                state->state
            );
            
            plan_sequence.emplace_back(
                state->primitive_idx,
                state->push_steps,
                global_pose
            );
        }
        
        // Cleanup before returning
        for (auto state : all_states) {
            delete state;
        }
        
        return plan_sequence;
    }

private:
    static std::vector<double> transform_to_local_frame(
        const std::vector<double>& reference,
        const std::vector<double>& target
    ) {
        double dx = target[0] - reference[0];
        double dy = target[1] - reference[1];
        double dtheta = normalize_angle(target[2] - reference[2]);  // Normalize angle difference
        
        // Rotate point by -reference[2]
        double cos_ref = std::cos(-reference[2]);
        double sin_ref = std::sin(-reference[2]);
        
        return {
            dx * cos_ref - dy * sin_ref,
            dx * sin_ref + dy * cos_ref,
            dtheta
        };
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
            normalize_angle(state[2] + yaw)  // Normalize the resulting angle
        };
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
            normalize_angle(reference[2] + local[2])  // Normalize combined angle
        };
    }

    static double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
};

} // namespace prx 