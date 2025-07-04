#pragma once
#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/mujoco/mj_simulator.hpp"
#include "motion_primitive_generator.hpp"
#include "best_first_search_planner.hpp"
#include "environment.hpp"
#include <nlohmann/json.hpp>

namespace prx {


struct ActionStep {
    std::string object_name;
    std::vector<double> goal_state;
    int edge_idx;
    int push_steps;
    std::vector<double> qpos;
    std::unique_ptr<MotionPrimitiveGenerator::PushState> push_state;
};

using json = nlohmann::json;

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
    PushController(NAMOEnvironment& env, bool visualize = false, const int control_steps = 500, const double scaling = 0.5, std::function<double(const std::vector<double>&, const std::vector<double>&, const int)> get_distance = nullptr, std::function<bool(const std::vector<double>&, const std::vector<double>&, const int)> is_goal_reached_fn = nullptr, const int mpc_steps_limit = 20, const int best_first_expansion_limit = 50) 
        :visualize(visualize), env(env), control_steps(control_steps), scaling(scaling), get_distance(get_distance), is_goal_reached_fn(is_goal_reached_fn), mpc_steps_limit(mpc_steps_limit), best_first_expansion_limit(best_first_expansion_limit)  { 
        
        // Get movable objects from environment
        movable_objects = env.get_movable_objects();
        robot_size = env.get_robot_size();
        
    }

    std::tuple<std::unique_ptr<MotionPrimitiveGenerator::PushState>, json> execute_primitive(std::vector<double> qpos_vec, const std::string& object_name, int push_steps, int edge_idx) {
        env.set_zero_velocity();
        env.set_qpos(qpos_vec);
        env.step_simulation();
        // set the qpos here

        auto object_state = env.get_object_state(object_name);
        auto [all_edge_points, all_mid_points] = get_object_edge_points_all();
        auto push_points = MotionPrimitiveGenerator::transform_points(all_edge_points[object_name], object_state->position, object_state->quaternion);
        auto mid_points = MotionPrimitiveGenerator::transform_points(all_mid_points[object_name], object_state->position, object_state->quaternion);

        // Create a smart pointer to PushState
        auto push_state = std::make_unique<MotionPrimitiveGenerator::PushState>();
        push_state->edge_idx = edge_idx;

        space_point_t control_point = env.get_control_space_point();

        json control_sequence = json::array();

        for (int i = 0; i < push_steps; i++) {
            
            if (i == 0) { // at the first pust step, reset the robot position to the push point
                env.set_robot_position(push_points[edge_idx]);
                push_state->edge_idx = edge_idx;
            }
            env.set_zero_velocity();
            env.step_simulation();
            for (int j = 0; j < control_steps; j++) {
                // TODO: add the states to the push_state for refined data collection.
                auto current_object_state = env.get_object_state(object_name);
                auto current_push_points = MotionPrimitiveGenerator::transform_points(all_edge_points[object_name], current_object_state->position, current_object_state->quaternion);
                auto current_mid_points = MotionPrimitiveGenerator::transform_points(all_mid_points[object_name], current_object_state->position, current_object_state->quaternion);


                auto robot_state = env.get_robot_state();
                control_sequence.push_back({
                    {"position", {robot_state->position[0], robot_state->position[1]}},
                });
                
                // update the push_state and generate new control
                push_state->current_edge_point = current_push_points[edge_idx]; // TODO: change to current robot position
                push_state->current_mid_point = current_mid_points[edge_idx];

                auto control = MotionPrimitiveGenerator::compute_control1(*push_state, scaling);
                control_point->at(0) = control[0];
                control_point->at(1) = control[1];
                env.step(control_point, 0.01);
                current_object_state = env.get_object_state(object_name);
            }
            env.set_zero_velocity();
            env.step_simulation();

            auto robot_state = env.get_robot_state();
            control_sequence.push_back({
                {"position", {robot_state->position[0], robot_state->position[1]}},
            });
        }
        return std::make_tuple(std::move(push_state), control_sequence);
    }

    // bool execute_push_action(const std::string& object_name, const std::vector<int>& allowed_primitive_indices, const std::vector<double>& goal_state) {
    //     // Get the current state of the object and call it start_state for the control planner
    //     const auto& object_info = env.get_object_info(object_name);
    //     const int symmetry_rotations = object_info->symmetry_rotations;

    //     auto object_state = env.get_object_state(object_name);
    //     std::vector<double> start_state = {
    //         object_state->position[0], object_state->position[1], 0.0, 
    //         object_state->quaternion[0], object_state->quaternion[1], 
    //         object_state->quaternion[2], object_state->quaternion[3]
    //     };

    //     std::vector<double> start_pose = {
    //         start_state[0], start_state[1], 
    //         quaternion_to_yaw({start_state[3], start_state[4], start_state[5], start_state[6]}, true)
    //     };
        
    //     std::vector<double> goal_pose = {
    //         goal_state[0], goal_state[1], 
    //         quaternion_to_yaw({goal_state[3], goal_state[4], goal_state[5], goal_state[6]}, true)
    //     };
        
    //     int num_steps = 0;

    //     while(num_steps < mpc_steps_limit) {
    //         // Generate the control plan which is a sequence of motion primitives
    //         std::vector<PlanStep> control_plan = compute_control_plan(
    //             object_name, start_pose, goal_pose, allowed_primitive_indices, symmetry_rotations);
            
    //         if (control_plan.empty()) {
    //             std::cout << "No control plan found for object: " << object_name << std::endl;
    //             return false;
    //         }

    //         for (int plan_step = 0; plan_step < control_plan.size(); plan_step++) {
    //             PlanStep& step = control_plan[plan_step];

    //             if (step.push_steps == 0) {
    //                 continue;
    //             }

    //             space_point_t qpos = env.get_qpos();
    //             std::vector<double> qpos_vec;
    //             env.get_state_space()->copy_vector_from_point(qpos_vec, qpos);

    //             // Execute primitive without storing the push state
    //             execute_primitive(qpos_vec, object_name, step.push_steps, step.edge_idx);
    //             break;  // Execute only the first non-zero push step
    //         }
            
    //         // Update object state after execution
    //         object_state = env.get_object_state(object_name);
    //         start_state = {
    //             object_state->position[0], object_state->position[1], 0.0, 
    //             object_state->quaternion[0], object_state->quaternion[1], 
    //             object_state->quaternion[2], object_state->quaternion[3]
    //         };
    //         start_pose = {
    //             start_state[0], start_state[1], 
    //             quaternion_to_yaw({start_state[3], start_state[4], start_state[5], start_state[6]}, true)
    //         };
            
    //         num_steps++;

    //         if (is_goal_reached_fn(start_pose, goal_pose, symmetry_rotations)) {
    //             return true;
    //         }
    //     }
        
    //     return false;
    // }

    std::tuple<std::unique_ptr<ActionStep>, bool> execute_push_action(const std::string& object_name, const std::vector<int>& allowed_primitive_indices, const std::vector<double>& goal_state) {


        // my skill is parameterized by the object name, allowed push points, and goal_state.
        
        // get the current state of the object and call it start_state for the control planner
        const auto& object_info = env.get_object_info(object_name);
        const int symmetry_rotations = object_info->symmetry_rotations;

        auto object_state = env.get_object_state(object_name);
        std::vector<double> start_state = {object_state->position[0], object_state->position[1], 0.0, object_state->quaternion[0], object_state->quaternion[1], object_state->quaternion[2], object_state->quaternion[3]};

        std::vector<double> start_pose = {start_state[0], start_state[1], 
                                        quaternion_to_yaw({start_state[3], start_state[4], 
                                                         start_state[5], start_state[6]}, true)};
        
        std::vector<double> goal_pose = {goal_state[0], goal_state[1], 
                                       quaternion_to_yaw({goal_state[3], goal_state[4], 
                                                        goal_state[5], goal_state[6]}, true)};
        int num_steps = 0;

        std::unique_ptr<ActionStep> action_step = nullptr;
        while(num_steps < mpc_steps_limit) {


            

            // generate the control plan which is a sequence of motion primitives
            std::vector<PlanStep> control_plan = compute_control_plan(object_name, start_pose, goal_pose, allowed_primitive_indices, symmetry_rotations);
            
            if (control_plan.empty()) {
                std::cout << "No control plan found for object: " << object_name << std::endl;
                return std::make_tuple(std::move(action_step), false);
            }


            // std::cout << "Control plan: " << control_plan.size() << std::endl;

            // std::chrono::high_resolution_clock::time_point control_start_timer = std::chrono::high_resolution_clock::now();
            for (int plan_step = 0; plan_step < control_plan.size(); plan_step++) {
                PlanStep& step = control_plan[plan_step];

                space_point_t qpos = env.get_qpos();
                std::vector<double> qpos_vec;
                env.get_state_space()->copy_vector_from_point(qpos_vec, qpos); // destination vector, source point

                // Execute primitive and get push state
                auto [push_state, control_sequence] = execute_primitive(qpos_vec, object_name, step.push_steps, step.edge_idx);
                
                // std::cout <<  "Plan steps: " << plan_step << " " << step.push_steps << "\n";
                // execute the first non-zero push step from the control plan
                if (step.push_steps != 0) {
                    action_step = std::make_unique<ActionStep>();
                    action_step->object_name = object_name;
                    // action_step->allowed_primitive_indices = *allowed_primitive_indices;
                    action_step->goal_state = std::vector<double>(goal_state);
                    action_step->edge_idx = step.edge_idx;
                    action_step->push_steps = step.push_steps;
                    action_step->qpos = qpos_vec;
                    action_step->push_state = std::move(push_state);
                    break; // this is what forces the control plan to be executed only once;
                }
            }
            
            object_state = env.get_object_state(object_name);
            start_state = {object_state->position[0], object_state->position[1], 0.0, object_state->quaternion[0], object_state->quaternion[1], object_state->quaternion[2], object_state->quaternion[3]};
            start_pose = {start_state[0], start_state[1], quaternion_to_yaw({start_state[3], start_state[4], start_state[5], start_state[6]}, true)};
            
            num_steps++;

            if (is_goal_reached_fn(start_pose, goal_pose, symmetry_rotations)) {
                return std::make_tuple(std::move(action_step), true);
            }
        }
        return std::make_tuple(std::move(action_step), false);
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
        bool visualize_primitives = false) {

        // for each movable object, generate motion primitives and store them in a map

        auto robot_info = env.get_robot_info();
        std::array<double, 3> robot_size = {robot_info.size[0], robot_info.size[1], robot_info.size[2]};

        if (robot_size[1] == 0.0) {
            robot_size[1] = robot_size[0];
        }

        
        // NAMOEnvironment::ObjectInfo generic_object;
        // generic_object.name = "generic";
        // generic_object.position = {0.0, 0.0, 0.0};
        // generic_object.size = {0.35, 0.35, 0.2};
        // generic_object.quaternion = {1.0, 0.0, 0.0, 0.0};
        

        for (const auto& obj : movable_objects) {
            auto primitives = MotionPrimitiveGenerator::generate_primitives(
                robot_size, base_config_path, visualize_primitives, push_steps, control_steps, scaling);
            
            if (generic_primitive.empty()) {
                generic_primitive = primitives;
                break;
            }

            // if (!primitives.empty()) {
            //     all_primitives[obj.name] = primitives;
            // }
        }

        // std::cout << "Preprocessed motion primitives for " << all_primitives.size() << " objects" << std::endl;
    }

    
    std::pair<std::unordered_map<std::string, std::vector<std::array<double, 2>>>, std::unordered_map<std::string, std::vector<std::array<double, 2>>>> get_object_edge_points_all() {
        std::unordered_map<std::string, std::vector<std::array<double, 2>>> edge_points;
        std::unordered_map<std::string, std::vector<std::array<double, 2>>> mid_points;
        for (const auto& obj : movable_objects) {
            auto [edge_points_vec, mid_points_vec] = MotionPrimitiveGenerator::generate_edge_points(obj.position, obj.size, obj.quaternion, robot_size);
            edge_points[obj.name] = edge_points_vec;
            mid_points[obj.name] = mid_points_vec;
        }
        return {edge_points, mid_points};
    }


    void execute_action_step(const ActionStep& action_step) {
        execute_primitive(action_step.qpos, action_step.object_name, action_step.push_steps, action_step.edge_idx);
    }

    // for object, compute the full control plan from start state to goal state assuming there is no other object in the environment
    std::vector<PlanStep> compute_control_plan(
        const std::string& object_name, 
        const std::vector<double>& start_pose, 
        const std::vector<double>& goal_pose,
        const std::vector<int>& allowed_primitive_indices,
        const int& symmetry_rotations) {
        
        // Get the motion primitives for the object
        auto primitives = generic_primitive; // all_primitives[object_name];

        // Use motion planner to find sequence of primitives
        return GreedyBestFirstSearchPlanner::plan_push_sequence(
            start_pose, 
            goal_pose, 
            primitives, 
            allowed_primitive_indices,
            symmetry_rotations,
            best_first_expansion_limit,
            get_distance,
            is_goal_reached_fn
        );
    }

    // std::unordered_map<std::string, std::vector<MotionPrimitive>> get_all_primitives() {
    //     return all_primitives;
    // }

    /**
     * @brief Save all motion primitives to a JSON file
     * 
     * @param filename Path to the output JSON file
     */
    void save_primitives_to_json(const std::string& filename) {
        json primitives_json;
        
        // Create directory if it doesn't exist
        std::filesystem::path file_path(filename);
        std::filesystem::create_directories(file_path.parent_path());
        
        // Convert all primitives to JSON
        for (const auto& [obj_name, primitives] : all_primitives) {
            json obj_primitives = json::array();
            
            for (const auto& primitive : primitives) {
                json prim_json;
                
                // Store primitive data
                prim_json["edge_idx"] = primitive.edge_idx;
                prim_json["push_steps"] = primitive.push_steps;
                prim_json["control_steps"] = primitive.control_steps;
                prim_json["scaling"] = primitive.scaling;
                
                // Store position
                prim_json["position"] = primitive.position;
                
                // Store quaternion
                prim_json["quaternion"] = {
                    primitive.quaternion[0],
                    primitive.quaternion[1],
                    primitive.quaternion[2],
                    primitive.quaternion[3]
                };
                
                // Store edge point and mid point
                prim_json["edge_point"] = {
                    primitive.edge_point[0],
                    primitive.edge_point[1]
                };
                
                prim_json["mid_point"] = {
                    primitive.mid_point[0],
                    primitive.mid_point[1]
                };
                
                obj_primitives.push_back(prim_json);
            }
            
            primitives_json[obj_name] = obj_primitives;
        }
        
        // Write to file
        std::ofstream output_file(filename);
        if (output_file.is_open()) {
            output_file << primitives_json.dump(4); // Pretty print with 4-space indentation
            output_file.close();
            std::cout << "Successfully saved primitives to: " << filename << std::endl;
        } else {
            std::cerr << "Error: Could not open file " << filename << " for writing" << std::endl;
        }
    }

    /**
     * @brief Load motion primitives from a JSON file
     * 
     * @param filename Path to the JSON file
     * @return bool True if primitives were successfully loaded
     */
    bool load_primitives_from_json(const std::string& filename) {
        try {
            // Check if file exists
            if (!std::filesystem::exists(filename)) {
                std::cout << "Primitives file not found: " << filename << std::endl;
                return false;
            }
            
            // Read JSON file
            std::ifstream input_file(filename);
            if (!input_file.is_open()) {
                std::cerr << "Error: Could not open file " << filename << " for reading" << std::endl;
                return false;
            }
            
            json primitives_json;
            input_file >> primitives_json;
            input_file.close();
            
            // Clear existing primitives
            all_primitives.clear();
            
            // Parse JSON into motion primitives
            for (const auto& [obj_name, obj_primitives_json] : primitives_json.items()) {
                std::vector<MotionPrimitive> obj_primitives;
                
                for (const auto& prim_json : obj_primitives_json) {
                    MotionPrimitive primitive;
                    
                    // Load primitive data
                    primitive.edge_idx = prim_json["edge_idx"];
                    primitive.push_steps = prim_json["push_steps"];
                    primitive.control_steps = prim_json["control_steps"];
                    primitive.scaling = prim_json["scaling"];
                    
                    // Load position
                    auto pos_json = prim_json["position"];
                    // Directly assign to array since position is std::array<double, 2>
                    primitive.position[0] = pos_json[0];
                    primitive.position[1] = pos_json[1];
                    
                    // Load quaternion
                    auto quat_json = prim_json["quaternion"];
                    for (int i = 0; i < 4; i++) {
                        primitive.quaternion[i] = quat_json[i];
                    }
                    
                    // Load edge point and mid point
                    auto edge_point_json = prim_json["edge_point"];
                    primitive.edge_point[0] = edge_point_json[0];
                    primitive.edge_point[1] = edge_point_json[1];
                    
                    auto mid_point_json = prim_json["mid_point"];
                    primitive.mid_point[0] = mid_point_json[0];
                    primitive.mid_point[1] = mid_point_json[1];
                    
                    obj_primitives.push_back(primitive);
                }
                
                all_primitives[obj_name] = obj_primitives;
            }
            
            std::cout << "Successfully loaded primitives from: " << filename << std::endl;
            return true;
        } catch (const std::exception& e) {
            std::cerr << "Error loading primitives from JSON: " << e.what() << std::endl;
            return false;
        }
    }

private:
    bool visualize;
    std::unordered_map<std::string, std::vector<MotionPrimitive>> all_primitives;
    std::vector<MotionPrimitive> generic_primitive;
    std::vector<NAMOEnvironment::ObjectInfo> movable_objects;
    int control_steps;
    double scaling;
    int mpc_steps_limit;
    int best_first_expansion_limit;
    std::function<double(const std::vector<double>&, const std::vector<double>&, const int)> get_distance;
    std::function<bool(const std::vector<double>&, const std::vector<double>&, const int)> is_goal_reached_fn;
    NAMOEnvironment& env;
    std::array<double, 3> robot_size;
};

} // namespace prx 