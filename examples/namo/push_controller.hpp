#pragma once
#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/mujoco/mj_simulator.hpp"
#include "motion_primitive_generator.hpp"
#include "best_first_search_planner.hpp"
#include "environment.hpp"

namespace prx {

/**
 * @brief Controller class for pushing objects in the NAMO environment
 */
class PushController {
public:
    /**
     * @brief Construct a new Push Controller
     * 
     * @param env Reference to the NAMO environment
     * @param visualize Whether to enable visualization
     */
    PushController(NAMOEnvironment& env, bool visualize = false, std::function<double(const std::vector<double>&, const std::vector<double>&, const int)> get_distance = nullptr, std::function<bool(const std::vector<double>&, const std::vector<double>&, const int)> is_goal_reached_fn = nullptr, int mpc_steps_limit = 20, int best_first_expansion_limit = 50) 
        :visualize(visualize), env(env), get_distance(get_distance), is_goal_reached_fn(is_goal_reached_fn), mpc_steps_limit(mpc_steps_limit), best_first_expansion_limit(best_first_expansion_limit)  { 
        
        // Get movable objects from environment
        movable_objects = env.get_movable_objects();

    }

    bool execute_push_action(const std::string& object_name, const std::vector<int>& allowed_primitive_indices, const std::vector<double>& goal_state) {
        
        // get the current state of the object and call it start_state for the control planner
        const auto& object_info = env.get_object_info(object_name);
        const int symmetry_rotations = object_info->symmetry_rotations;

        env.reset();

        auto object_state = env.get_object_state(object_name);
        std::vector<double> start_state = {object_state->position[0], object_state->position[1], 0.0, object_state->quaternion[0], object_state->quaternion[1], object_state->quaternion[2], object_state->quaternion[3]};

        std::vector<double> start_pose = {start_state[0], start_state[1], 
                                        quaternion_to_yaw({start_state[3], start_state[4], 
                                                         start_state[5], start_state[6]}, true)};
        
        std::vector<double> goal_pose = {goal_state[0], goal_state[1], 
                                       quaternion_to_yaw({goal_state[3], goal_state[4], 
                                                        goal_state[5], goal_state[6]}, true)};
        int num_steps = 0;

        while(num_steps < mpc_steps_limit) {

            // generate the control plan which is a sequence of motion primitives
            std::vector<PlanStep> control_plan = compute_control_plan(object_name, start_pose, goal_pose, allowed_primitive_indices);
            space_point_t control_point = env.get_control_space_point();

            if (control_plan.empty()) {
                std::cout << "No control plan found for object: " << object_name << std::endl;
                return false;
            }

            for (int plan_step = 0; plan_step < control_plan.size(); plan_step++) {
                const PlanStep& step = control_plan[plan_step];
                
                // execute the primitive
                env.set_zero_velocity();
                for (int i = 0; i < 1; i++) {
                    env.step_simulation();
                }
                auto object_state = env.get_object_state(object_name);
                auto [all_edge_points, all_mid_points] = get_object_edge_points_all();
                auto push_points = MotionPrimitiveGenerator::transform_points(all_edge_points[object_name], object_state->position, object_state->quaternion);
                auto mid_points = MotionPrimitiveGenerator::transform_points(all_mid_points[object_name], object_state->position, object_state->quaternion);

                MotionPrimitiveGenerator::PushState push_state;
                push_state.edge_idx = step.edge_idx;

                for (int i = 0; i < step.push_steps; i++) {
                    if (i == 0) {
                        env.set_robot_position(push_points[step.edge_idx]);
                        push_state.edge_idx = step.edge_idx;
                    }
                    env.set_zero_velocity();
                    env.step_simulation();
                    for (int j = 0; j < control_steps; j++) {
                        auto current_object_state = env.get_object_state(object_name);
                        auto current_push_points = MotionPrimitiveGenerator::transform_points(all_edge_points[object_name], current_object_state->position, current_object_state->quaternion);
                        auto current_mid_points = MotionPrimitiveGenerator::transform_points(all_mid_points[object_name], current_object_state->position, current_object_state->quaternion);

                        // update the push_state and generate new control
                        push_state.current_edge_point = current_push_points[step.edge_idx];
                        push_state.current_mid_point = current_mid_points[step.edge_idx];

                        auto control = MotionPrimitiveGenerator::compute_control1(push_state, scaling);
                        control_point->at(0) = control[0];
                        control_point->at(1) = control[1];
                        env.step(control_point, 0.01);

                        current_object_state = env.get_object_state(object_name);
                    }
                    env.set_zero_velocity();
                    env.step_simulation();
                }
                if (step.push_steps != 0) {
                    break;
                }
            }

            object_state = env.get_object_state(object_name);
            start_state = {object_state->position[0], object_state->position[1], 0.0, object_state->quaternion[0], object_state->quaternion[1], object_state->quaternion[2], object_state->quaternion[3]};
            start_pose = {start_state[0], start_state[1], quaternion_to_yaw({start_state[3], start_state[4], start_state[5], start_state[6]}, true)};
            num_steps++;

            if (is_goal_reached_fn(start_pose, goal_pose, symmetry_rotations)) {
                return true;
            }
        }
        return false;
    }

    /**
     * @brief Generate motion primitives for all movable objects in the environment
     * 
     * @param push_steps Number of pushing steps to simulate for each edge point
     * @param control_steps Number of simulation steps for each push action
     * @param scaling Scaling factor for the pushing force
     * @return std::unordered_map<std::string, std::vector<MotionPrimitive>> 
     *         Map of object names to their motion primitives
     */
    void preprocess_all_motion_primitives(
        std::string base_config_path,
        int push_steps = 20,
        int control_steps = 500,
        double scaling = 0.5,
        bool visualize_primitives = false) {

        // for each movable object, generate motion primitives and store them in a map
        this->control_steps = control_steps;
        this->scaling = scaling;

        for (const auto& obj : movable_objects) {
            auto primitives = MotionPrimitiveGenerator::generate_primitives(
                obj, base_config_path, visualize_primitives, push_steps, control_steps, scaling);
            
            if (!primitives.empty()) {
                all_primitives[obj.name] = primitives;
            }
        }

        std::cout << "Preprocessed motion primitives for " << all_primitives.size() << " objects" << std::endl;
    }

    
    std::pair<std::unordered_map<std::string, std::vector<std::array<double, 2>>>, std::unordered_map<std::string, std::vector<std::array<double, 2>>>> get_object_edge_points_all() {
        std::unordered_map<std::string, std::vector<std::array<double, 2>>> edge_points;
        std::unordered_map<std::string, std::vector<std::array<double, 2>>> mid_points;
        for (const auto& obj : movable_objects) {
            auto [edge_points_vec, mid_points_vec] = MotionPrimitiveGenerator::generate_edge_points(obj.position, obj.size, obj.quaternion);
            edge_points[obj.name] = edge_points_vec;
            mid_points[obj.name] = mid_points_vec;
        }
        return {edge_points, mid_points};
    }



    // for object, compute the full control plan from start state to goal state assuming there is no other object in the environment
    std::vector<PlanStep> compute_control_plan(
        const std::string& object_name, 
        const std::vector<double>& start_pose, 
        const std::vector<double>& goal_pose,
        const std::vector<int>& allowed_primitive_indices) {
        
        // Get the motion primitives for the object
        auto primitives = all_primitives[object_name];

        // Use motion planner to find sequence of primitives
        return GreedyBestFirstSearchPlanner::plan_push_sequence(
            start_pose, 
            goal_pose, 
            primitives, 
            allowed_primitive_indices,
            best_first_expansion_limit,
            get_distance,
            is_goal_reached_fn
        );
    }

    std::unordered_map<std::string, std::vector<MotionPrimitive>> get_all_primitives() {
        return all_primitives;
    }

private:
    bool visualize;
    std::unordered_map<std::string, std::vector<MotionPrimitive>> all_primitives;
    std::vector<NAMOEnvironment::ObjectInfo> movable_objects;
    int control_steps;
    double scaling;
    int mpc_steps_limit;
    int best_first_expansion_limit;
    std::function<double(const std::vector<double>&, const std::vector<double>&, const int)> get_distance;
    std::function<bool(const std::vector<double>&, const std::vector<double>&, const int)> is_goal_reached_fn;
    NAMOEnvironment& env;
};

} // namespace prx 