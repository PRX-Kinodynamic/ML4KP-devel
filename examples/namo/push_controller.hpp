#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/mujoco/mj_simulator.hpp"
#include "motion_primitive_generator.hpp"
#include "best_first_search_planner.hpp"

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
     * @param base_config_path Path to the base configuration XML file used for motion primitive generation
     * @param visualize Whether to enable visualization
     */
    PushController(NAMOEnvironment& env, const std::string& base_config_path, bool visualize = false) 
        : base_config_path(base_config_path), visualize(visualize) {
        
        // Get movable objects from environment
        movable_objects = env.get_movable_objects();
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
        int push_steps = 20,
        int control_steps = 500,
        double scaling = 0.5) {

        // for each movable object, generate motion primitives and store them in a map
        
        for (const auto& obj : movable_objects) {
            auto primitives = MotionPrimitiveGenerator::generate_primitives(
                obj, base_config_path, visualize, push_steps, control_steps, scaling);
            
            if (!primitives.empty()) {
                all_primitives[obj.name] = primitives;
            }
        }

        std::cout << "Preprocessed motion primitives for " << all_primitives.size() << " objects" << std::endl;
    }

    
    std::unordered_map<std::string, std::vector<std::array<double, 2>>> get_object_edge_points_all() {
        std::unordered_map<std::string, std::vector<std::array<double, 2>>> edge_points;
        for (const auto& obj : movable_objects) {
            auto [points, mid_points] = MotionPrimitiveGenerator::generate_edge_points(obj.position, obj.size, obj.quaternion);
            edge_points[obj.name] = points;
        }
        return edge_points;
    }






    // for object, compute the full control plan from start state to goal state assuming there is no other object in the environment
    std::vector<int> compute_control_plan(
        const std::string& object_name, 
        const std::vector<double>& start_state, 
        const std::vector<double>& goal_state,
        const std::vector<int>& allowed_primitive_indices) {
        
        std::cout << "Computing control plan for object: " << object_name << std::endl;

        // Get the motion primitives for the object
        auto primitives = all_primitives[object_name];
        
        // Convert states to x, y, theta format if needed
        std::vector<double> start_pose = {start_state[0], start_state[1], 
                                        quaternion_to_yaw({start_state[3], start_state[4], 
                                                         start_state[5], start_state[6]})};
        
        std::vector<double> goal_pose = {goal_state[0], goal_state[1], 
                                       quaternion_to_yaw({goal_state[3], goal_state[4], 
                                                        goal_state[5], goal_state[6]})};

        // Use motion planner to find sequence of primitives
        return GreedyBestFirstSearchPlanner::plan_push_sequence(
            start_pose, 
            goal_pose, 
            primitives, 
            allowed_primitive_indices
        );
    }

    std::unordered_map<std::string, std::vector<MotionPrimitive>> get_all_primitives() {
        return all_primitives;
    }

private:
    std::string base_config_path;
    bool visualize;
    std::unordered_map<std::string, std::vector<MotionPrimitive>> all_primitives;
    std::vector<NAMOEnvironment::ObjectInfo> movable_objects;
};

} // namespace prx 