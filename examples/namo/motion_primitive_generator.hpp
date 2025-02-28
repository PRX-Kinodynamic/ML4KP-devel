#pragma once
#include "mujoco_simple_wrapper.hpp"
#include <filesystem>
#include <fstream>
#include "namo_utility.hpp"

namespace prx {

struct ObjectInfo {
    std::array<double, 3> size;  // [x, y, z]
    int symmetry_rotations;    // 2 or 4 based on object shape
    
    // Add default constructor
    ObjectInfo() : size({1.0, 1.0, 1.0}), symmetry_rotations(4) {}
    
    ObjectInfo(const std::array<double, 3>& size) : size(size) {
        // If x and y dimensions are within 5% of each other, 4-way symmetry
        double size_ratio = std::max(size[0], size[1]) / std::min(size[0], size[1]);
        symmetry_rotations = (size_ratio < 1.05) ? 4 : 2;
    }
};

/**
 * @brief Structure to store a single motion primitive
 * 
 * Contains the state information and control parameters for a pushing motion
 */
struct MotionPrimitive {
    std::vector<double> position;
    std::array<double, 4> quaternion;
    int edge_idx;
    std::array<double, 2> edge_point;
    std::array<double, 2> mid_point;
    int push_steps;
    int control_steps;
    double scaling;
    ObjectInfo object_info;  // Add object info

    // Add default constructor
    MotionPrimitive() : 
        position(std::vector<double>(3, 0.0)),
        quaternion({1.0, 0.0, 0.0, 0.0}),
        edge_idx(0),
        edge_point({0.0, 0.0}),
        mid_point({0.0, 0.0}),
        push_steps(0),
        control_steps(0),
        scaling(1.0),
        object_info() {}
};

/**
 * @brief Static class for generating motion primitives for pushing objects
 */
class MotionPrimitiveGenerator {
public:
    /**
     * @brief Generate motion primitives for a single object
     * 
     * @param obj Object information including position, size, and orientation
     * @param base_config_path Path to base configuration XML
     * @param temp_folder Path to temp folder for saving trajectories
     * @param visualize Whether to enable visualization
     * @param push_steps Number of pushing steps
     * @param control_steps Number of simulation steps per push
     * @param scaling Force scaling factor
     * @return std::vector<MotionPrimitive> Vector of generated motion primitives
     */
    static std::vector<MotionPrimitive> generate_primitives(
        const NAMOEnvironment::ObjectInfo& obj,
        const std::string& base_config_path,
        bool visualize = false,
        int push_steps = 20,
        int control_steps = 500,
        double scaling = 0.5) {
        
        std::string temp_folder = "namo_trajectories";
        // Create temp directory if it doesn't exist
        if (!std::filesystem::exists(temp_folder)) {
            std::filesystem::create_directories(temp_folder);
        }
        
        // Create simulation environment
        auto sim = setup_simulation(obj, base_config_path, visualize);
        if (!sim) return {};
        std::array<double, 3> base_robot_pos;
        sim->getBodyPosition("robot", base_robot_pos);
        
        std::array<double, 3> base_pos;
        sim->getBodyPosition(obj.name, base_pos);

        sim->setZeroVelocity();
        for (int i = 0; i < 5; i++) {
            sim->step();
        }

        sim->getBodyPosition("robot", base_robot_pos);
        sim->getBodyPosition(obj.name, base_pos);
        for (int i = 0; i < 1; i++) {
            sim->step();
        }
        // Generate primitives
        return generate_primitives_for_object(sim, obj, push_steps, control_steps, scaling, base_robot_pos, temp_folder);
    }


    // Add new helper struct to maintain push state
    struct PushState {
        int edge_idx;
        std::array<double, 2> initial_edge_point;
        std::array<double, 2> initial_mid_point;
        
        // Current points based on object position
        std::array<double, 2> current_edge_point;
        std::array<double, 2> current_mid_point;
    };

    /**
     * @brief Compute control based on current push state
     */
    static std::array<double, 2> compute_control1(const PushState& state, double scaling) {
        double dx = state.current_mid_point[0] - state.current_edge_point[0];
        double dy = state.current_mid_point[1] - state.current_edge_point[1];
        
        double angle = std::atan2(dy, dx);

        return {
            scaling * std::cos(angle),
            scaling * std::sin(angle)
        };
    }

    /**
     * @brief Update push state based on current object position
     */
    static void update_push_state(
        PushState& state,
        const std::array<double, 3>& obj_pos,
        const std::array<double, 3>& obj_size,
        const std::array<double, 4>& obj_quat) {
        
        auto [untransformed_edge_points, untransformed_mid_points] = generate_edge_points(obj_pos, obj_size, obj_quat);
        
        // transform edge points and mid points to world coordinates
        auto edge_points = transform_points(untransformed_edge_points, obj_pos, obj_quat);
        auto mid_points = transform_points(untransformed_mid_points, obj_pos, obj_quat);


        // Update current points while maintaining same edge_idx
        state.current_edge_point = edge_points[state.edge_idx];
        state.current_mid_point = mid_points[state.edge_idx];

    }

    /**
     * @brief Create and setup simulation environment
     */
    static std::unique_ptr<MujocoWrapper> setup_simulation(
        const NAMOEnvironment::ObjectInfo& obj,
        const std::string& base_config_path,
        bool visualize) {
        
        try {
            // Parse base XML into mjSpec
            std::array<char, 1000> error;
            mjSpec* spec = mj_parseXML(base_config_path.c_str(), nullptr, error.data(), error.size());
            if (!spec) {
                std::cerr << "XML parse error: " << error.data() << std::endl;
                return nullptr;
            }

            // Add robot to world body
            mjsBody* world = mjs_findBody(spec, "world");

            // Add object body and geom
            mjsBody* obj_body = mjs_findBody(spec, "obstacle_1_movable");
            mjsGeom* obj_geom = mjs_asGeom(mjs_findElement(spec, mjOBJ_GEOM, "obstacle_1_movable"));
            mjs_setString(obj_body->name, obj.name.c_str());
            mjs_setString(obj_geom->name, obj.name.c_str());

            obj_geom->pos[0] = 0.0;
            obj_geom->pos[1] = 0.0;
            obj_geom->pos[2] = obj.position[2];
            for (int i = 0; i < 3; i++) {
                obj_geom->size[i] = obj.size[i];
            }
         
           
            mjModel* m = mj_compile(spec, nullptr);
           
            if (!m) {
                std::cerr << "Model compilation failed" << std::endl;
                std::cerr << "Error: " << mjs_getError(spec) << std::endl;
                
                return nullptr;
            }
            int error_sz;
            char error_buffer[1000];
            mj_saveXML(spec, "test.xml", error_buffer, error_sz);

            // Create wrapper with compiled model
            return std::make_unique<MujocoWrapper>(m, visualize);
        }
        catch (const std::exception& e) {
            std::cerr << "Failed to setup simulation: " << e.what() << std::endl;
            return nullptr;
        }
    }

    /**
     * @brief Generate primitives for a specific object
     */
    static std::vector<MotionPrimitive> generate_primitives_for_object(
        std::unique_ptr<MujocoWrapper>& sim,
        const NAMOEnvironment::ObjectInfo& obj,
        int push_steps,
        int control_steps,
        double scaling,
        const std::array<double, 3>& base_robot_pos,
        const std::string& temp_folder) {
        
        std::vector<MotionPrimitive> primitives;
        std::array<double, 3> obj_pos;
        std::array<double, 4> obj_quat;
        sim->getBodyPosition(obj.name, obj_pos);
        sim->getBodyQuaternion(obj.name, obj_quat);

        auto [edge_points, mid_points] = generate_edge_points(obj_pos, obj.size, obj_quat);

        for (int edge_idx = 0; edge_idx < edge_points.size(); edge_idx++) {
            PushState initial_state{
                edge_idx,
                edge_points[edge_idx],
                mid_points[edge_idx],
                edge_points[edge_idx],
                mid_points[edge_idx]
            };
            
            auto trajectory = simulate_push(sim, obj, initial_state, 
                                         push_steps, control_steps, scaling, base_robot_pos);
            
            // Save trajectory to file
            std::string filename = temp_folder + "/" + obj.name + "_edge" + 
                                 std::to_string(edge_idx) + ".txt";
            save_trajectory(trajectory, filename);
            
            primitives.insert(primitives.end(), trajectory.begin(), trajectory.end());
            sim->reset();
        }

        return primitives;
    }

    /**
     * @brief Simulate pushing motion from an edge point
     */
    static std::vector<MotionPrimitive> simulate_push(
        std::unique_ptr<MujocoWrapper>& sim,
        const NAMOEnvironment::ObjectInfo& obj,
        const PushState& initial_state,
        int push_steps,
        int control_steps,
        double scaling,
        const std::array<double, 3>& base_robot_pos) {

        std::vector<MotionPrimitive> trajectory;
        PushState current_state = initial_state;
        
        
        // Initial robot positioning
        std::array<double, 3> robot_pos = {
            current_state.current_edge_point[0] - base_robot_pos[0], 
            current_state.current_edge_point[1] - base_robot_pos[1], 
            obj.size[2]
        };
        sim->setRobotVelocity({0, 0});
        sim->setRobotPosition(robot_pos);
        sim->setZeroControl();
        sim->step();
        
        for (int step = 1; step <= push_steps; step++) {
            sim->setZeroVelocity();
            sim->setZeroControl();
            sim->step();
            std::array<double, 3> obj_pos;
            std::array<double, 4> obj_quat;
            sim->getBodyPosition(obj.name, obj_pos);
            sim->getBodyQuaternion(obj.name, obj_quat);
            update_push_state(current_state, obj_pos, obj.size, obj_quat);
            
            for (int i = 0; i < control_steps; i++) {
                // Get current object state
                // Update push state based on current object position
                
                // Compute and apply control
                auto ctrl = compute_control1(current_state, scaling);
                sim->setControl(ctrl.data(), 2);
                
                // Step simulation
                sim->step();
                sim->getBodyPosition(obj.name, obj_pos);
                sim->getBodyQuaternion(obj.name, obj_quat);
                update_push_state(current_state, obj_pos, obj.size, obj_quat);
            }
            // Record state
            MotionPrimitive primitive;
            primitive.position = {obj_pos[0], obj_pos[1]};
            primitive.quaternion = {obj_quat[0], obj_quat[1], obj_quat[2], obj_quat[3]};
            primitive.edge_point = current_state.current_edge_point;
            primitive.mid_point = current_state.current_mid_point;
            primitive.push_steps = step;
            primitive.control_steps = control_steps;
            primitive.scaling = scaling;
            primitive.edge_idx = current_state.edge_idx;
            ObjectInfo object_info(obj.size);
            primitive.object_info = object_info;
            trajectory.push_back(primitive);
        }

        return trajectory;
    }

    /**
     * @brief Generate edge points around an object for pushing
     * 
     * @param pos Object position
     * @param size Object size
     * @param rotation Object orientation as quaternion
     * @return std::pair<std::vector<std::array<double, 2>>, std::vector<std::array<double, 2>>> List of 2D points around object edges and mid points
     */
    static std::pair<std::vector<std::array<double, 2>>, std::vector<std::array<double, 2>>> generate_edge_points(
        const std::array<double, 3>& pos,
        const std::array<double, 3>& size,
        const std::array<double, 4>& rotation) {
        
        std::vector<std::array<double, 2>> points;
        double x = 0, y = 0;
        double w = size[0] - 0.05, d = size[1] - 0.05;
        double angle = 0.0; // quaternion_to_yaw(rotation, true);
        double offset = 0.1;

        // Generate 3 points on each edge
        std::vector<std::array<double, 2>> cedge_points = {{x - w, y + d + offset}, {x - w, y - d - offset}, {x, y + d + offset}, {x, y - d - offset}, {x + w, y + d + offset}, {x + w, y - d - offset}, {x + w + offset, y - d}, {x - w - offset, y - d}, {x + w + offset, y}, {x - w - offset, y}, {x + w + offset, y + d}, {x - w - offset, y + d}};
        // std::vector<std::array<double, 2>> edge_points = {{x, y + d + offset}, {x, y - d - offset}, {x + w + offset, y}, {x - w - offset, y}};


        // Apply rotation if needed
        for (const auto& edge_point : edge_points) {
            if (angle != 0) {
                std::array<double, 2> rotated_point = rotate_point(edge_point[0], edge_point[1], x, y, angle);
                points.push_back(rotated_point);
            } else {
                points.push_back(edge_point);
            }
        }

        // compute mid points
        std::vector<std::array<double, 2>> mid_points;
        for (int i = 0; i < points.size(); i++) {
            if (i % 2 == 0) {
                mid_points.push_back(get_mid_point(points[i], points[i+1]));
            }else{
                mid_points.push_back(get_mid_point(points[i], points[i-1]));
            }
        }
        return std::make_pair(points, mid_points);
    }

    static std::vector<std::array<double, 2>> transform_points(
        const std::vector<std::array<double, 2>>& points,
        const std::array<double, 3>& pos,
        const std::array<double, 4>& rotation) {
        
        std::vector<std::array<double, 2>> transformed_points;
        double angle = quaternion_to_yaw(rotation, true);
        
        for (const auto& point : points) {
            // First rotate around origin (0,0)
            std::array<double, 2> rotated_point = rotate_point(point[0], point[1], 0, 0, angle);
            
            // Then translate to world coordinates
            std::array<double, 2> transformed_point = {
                rotated_point[0] + pos[0],
                rotated_point[1] + pos[1]
            };
            
            transformed_points.push_back(transformed_point);
        }
        return transformed_points;
    }

    private:

    static void save_trajectory(const std::vector<MotionPrimitive>& trajectory, 
                              const std::string& filename) {
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Failed to open file: " << filename << std::endl;
            return;
        }

        for (const auto& primitive : trajectory) {
            file << primitive.position[0] << " " << primitive.position[1] << " "
                 << primitive.quaternion[0] << " " << primitive.quaternion[1] << " "
                 << primitive.quaternion[2] << " " << primitive.quaternion[3] << " "
                 << primitive.edge_point[0] << " " << primitive.edge_point[1] << " "
                 << primitive.mid_point[0] << " " << primitive.mid_point[1] << " "
                 << primitive.push_steps << " " << primitive.control_steps << " "
                 << primitive.scaling << "\n";
        }
        file.close();
    }
};

} // namespace prx 