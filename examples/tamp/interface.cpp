#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/progress_bar.hpp"
// #include "prx/mujoco/mj_simulator.hpp"
#include "push_environment.hpp"
#include "push_controller.hpp"

#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>
// #include <Eigen/Dense>
// #include <cmath>

using namespace prx;
using namespace boost::filesystem;
using json = nlohmann::json;

// TODO: move this to a utility file
void create_folder(std::string path)
{
    // std::cout << "Creating folder: " << path << std::endl;
    if (!exists(path))
    {
        create_directories(path);
    }
}

// TODO: move this to a utility file
int count_folders(const std::string& path) {
    return std::count_if(directory_iterator(path), directory_iterator{},
                        [](const directory_entry& entry) { return is_directory(entry); });
}

void save_data(std::string base_folder, trajectory_t* traj, trajectory_t* control_traj, 
               std::string xml_path, space_point_t goal_state, bool success, 
               int num_steps, const PushEnvironment& env)
{

    size_t last_slash = xml_path.find_last_of("/");
    size_t first_slash = xml_path.find_first_of("/");
    std::string env_folder = xml_path.substr(first_slash + 1, last_slash - first_slash - 1);

    // Create success/failure subfolder
    std::string folder = base_folder + "/" + env_folder + "/" + (success ? "success" : "failure");
    create_folder(folder);

    // Count existing folders in success/failure directory
    int folder_count = count_folders(folder);
    std::string save_folder = folder + "/" + std::to_string(folder_count);
    create_folder(save_folder);

    // Save trajectories
    traj->to_file(save_folder + "/trajectory.txt");
    // control_traj->to_file(save_folder + "/control_trajectory.txt");

    // Create JSON metadata
    json metadata;
    metadata["xml_path"] = xml_path;
    // Convert goal_state to vector since it doesn't have to_string()
    std::vector<double> goal_state_vec;
    env.get_state_space()->copy_vector_from_point(goal_state_vec, goal_state);
    metadata["goal_state"] = goal_state_vec;
    metadata["success"] = success;
    metadata["num_steps"] = num_steps;

    // Add environment geometry information
    metadata["environment"] = env.get_geom_info();  // You'll need to implement this in PushEnvironment

    // Save JSON metadata
    std::ofstream file(save_folder + "/metadata.json");
    file << metadata.dump(4);  // 4 spaces indentation
    file.close();
}

// Add this structure before main()
struct GoalResult {
    std::vector<double> position;
    bool success;
    int num_steps;
};

// Helper function to convert world coordinates to grid coordinates
std::pair<int, int> world_to_grid(double x, double y, double resolution, 
                                 double x_min, double y_min, int width, int height) {
    int grid_x = static_cast<int>((x - x_min) / resolution);
    int grid_y = static_cast<int>((y - y_min) / resolution);
    
    // Clamp values to grid bounds
    grid_x = std::max(0, std::min(grid_x, width - 1));
    grid_y = std::max(0, std::min(grid_y, height - 1));
    
    return {grid_x, grid_y};
}

// Function to draw a rotated rectangle on the grid
void draw_rectangle(Eigen::MatrixXd& grid, double cx, double cy, 
                   double width, double height, double angle,
                   double resolution, double x_min, double y_max) {
    int grid_width = grid.cols();
    int grid_height = grid.rows();
    
    // Convert angle to radians
    double theta = angle * M_PI / 180.0;
    double cos_t = cos(theta);
    double sin_t = sin(theta);
    
    // Sample points within the rectangle bounds
    for(int i = 0; i < grid_width; i++) {
        for(int j = 0; j < grid_height; j++) {
            // Convert grid coordinates to world coordinates
            double x = x_min + i * resolution;
            double y = y_max - j * resolution;
            
            // Translate point to origin
            double dx = x - cx;
            double dy = y - cy;
            
            // Rotate point
            double rx = dx * cos_t + dy * sin_t;
            double ry = -dx * sin_t + dy * cos_t;
            
            // Check if point is inside rectangle
            if(std::abs(rx) <= width/2 && std::abs(ry) <= height/2) {
                grid(j, i) = 1.0;
            }
        }
    }
}

void create_scene_images(const json& env_info, json& aggregate_data, 
                        double resolution=0.1) {
    // Define grid bounds (assuming -2 to 2 as in the environment)
    double x_min = -2.0, x_max = 2.0;
    double y_min = -2.0, y_max = 2.0;
    
    // Calculate grid dimensions
    int width = static_cast<int>((x_max - x_min) / resolution);
    int height = static_cast<int>((y_max - y_min) / resolution);
    
    // Create four grids: full scene, static objects, movable objects (no robot), and robot
    Eigen::MatrixXd full_scene = Eigen::MatrixXd::Zero(height, width);
    Eigen::MatrixXd static_mask = Eigen::MatrixXd::Zero(height, width);
    Eigen::MatrixXd movable_mask = Eigen::MatrixXd::Zero(height, width);
    Eigen::MatrixXd robot_mask = Eigen::MatrixXd::Zero(height, width);
    
    for(const auto& obj : env_info) {
        // Get object properties
        std::vector<double> pos = obj["pos"];
        std::vector<double> size = obj["size"];
        std::vector<double> quat = obj["quat"];
        std::string name = obj["name"];
        
        // Convert quaternion to euler angles (assuming Z-axis rotation)
        // For scalar-first quaternion [w, x, y, z]
        double angle = std::atan2(2.0 * (quat[0] * quat[3] + quat[1] * quat[2]),
                                 1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3])) * 180.0 / M_PI;
        
        // Remember: size is half-dimensions
        if(name.find("obstacle") != std::string::npos || name.find("robot") != std::string::npos) {
            draw_rectangle(full_scene, pos[0], pos[1], size[0]*2, size[1]*2, angle,
                         resolution, x_min, y_max);
            
            if(name.find("robot") != std::string::npos) {
                draw_rectangle(robot_mask, pos[0], pos[1], size[0]*2, size[1]*2, angle,
                             resolution, x_min, y_max);
            }
            else if(name.find("movable") != std::string::npos) {
                draw_rectangle(movable_mask, pos[0], pos[1], size[0]*2, size[1]*2, angle,
                             resolution, x_min, y_max);
            }
            else {
                draw_rectangle(static_mask, pos[0], pos[1], size[0]*2, size[1]*2, angle,
                             resolution, x_min, y_max);
            }
        }
    }
    
    // Convert Eigen matrices to vectors for JSON storage
    std::vector<std::vector<int>> full_scene_vec(height);
    std::vector<std::vector<int>> static_mask_vec(height);
    std::vector<std::vector<int>> movable_mask_vec(height);
    std::vector<std::vector<int>> robot_mask_vec(height);
    
    for(int i = 0; i < height; i++) {
        full_scene_vec[i].resize(width);
        static_mask_vec[i].resize(width);
        movable_mask_vec[i].resize(width);
        robot_mask_vec[i].resize(width);
        
        for(int j = 0; j < width; j++) {
            full_scene_vec[i][j] = full_scene(i, j) > 0.5 ? 1 : 0;
            static_mask_vec[i][j] = static_mask(i, j) > 0.5 ? 1 : 0;
            movable_mask_vec[i][j] = movable_mask(i, j) > 0.5 ? 1 : 0;
            robot_mask_vec[i][j] = robot_mask(i, j) > 0.5 ? 1 : 0;
        }
    }
    
    // Add to aggregate data
    aggregate_data["scene_images"] = {
        {"resolution", resolution},
        {"x_min", x_min},
        {"x_max", x_max},
        {"y_min", y_min},
        {"y_max", y_max},
        {"full_scene", full_scene_vec},
        {"static_mask", static_mask_vec},
        {"movable_mask", movable_mask_vec},
        {"robot_mask", robot_mask_vec}
    };
}

int main(int argc, char* argv[])
{
    // Check command line arguments
    if (argc < 2 || argc > 3) {
        std::cout << "Usage: " << argv[0] << " <parameter_file_path> [save_data=true]" << std::endl;
        return 1;
    }

    // Parse save_data flag (default to true if not provided)
    bool save_data_flag = true;
    if (argc == 3) {
        std::string save_arg = argv[2];
        if (save_arg == "false" || save_arg == "0") {
            save_data_flag = false;
        }
    }
    // Load parameters from command line argument
    param_loader params(argv[1]); 
    init_random(params["random_seed"].as<int>());
    
    std::string data_path = params["data_path"].as<std::string>();
    
    // Only create folders if we're saving data
    int folder_count = 0;
    if (save_data_flag) {
        create_folder(data_path);
        folder_count = count_folders(data_path);
    }

    double discretization = params["discretization"].as<double>();
    double duration = params["step_duration"].as<double>();
    int time_limit = params["time_limit"].as<int>();
    double goal_radius = params["goal_radius"].as<double>();

    std::string controller_mode = params["controller_mode"].as<std::string>();
    
    param_loader controller_params = params[controller_mode];

    int control_steps = time_limit / duration;
    
    int goal_count = 0;
    int success_count = 0;
    int num_steps = 0;

    // Initialize environment
    PushEnvironment env(params["xml_path"].as<std::string>(), 
                       params["visualize"].as<bool>(), goal_radius);
    
    if(params["goal_sampling_mode"].as<std::string>() == "random") {
        env.set_goal_sampling_mode(PushEnvironment::GoalSamplingMode::RANDOM);
        env.set_random_goals(params["num_random_goals"].as<int>());
    }
    else if (params["goal_sampling_mode"].as<std::string>() == "discretized") {
        env.set_goal_sampling_mode(PushEnvironment::GoalSamplingMode::DISCRETIZED);
        env.set_discretization(discretization);
    }
    else if (params["goal_sampling_mode"].as<std::string>() == "file") {
        env.set_goal_sampling_mode(PushEnvironment::GoalSamplingMode::FILE);
        env.load_goals_from_file(params["goals_file"].as<std::string>());
    }
    else {
        std::cout << "Invalid goal sampling mode" << std::endl;
        return 1;
    }

    // Initialize controller
    auto control_point = env.get_control_space_point(); // Get control point
    SimplePushController controller(controller_params, control_point);
    
    int total_goals = env.get_total_goals();
    progress_bar_t progress(total_goals, "Processing goals");

    // Initialize for forward propagation and data collection
    space_point_t current_state = env.get_current_state();
    trajectory_t* traj = new trajectory_t(env.get_state_space());
    trajectory_t* control_traj = new trajectory_t(env.get_control_space());
    
    // Create success/failure folders at startup only if saving data
    // if (save_data_flag) {
    //     create_folder(data_path + "/success");
    //     create_folder(data_path + "/failure");
    // }

    // Add this before the goal iteration loop
    std::vector<GoalResult> goal_results;
    
    // Iterate through all possible goals
    while (auto goal_state = env.next_goal()) {

        traj->clear();
        control_traj->clear();
        bool success = false;
        num_steps = 0;
        progress.update(++goal_count);

        env.add_pair(); // add collision pairs between the target movable obstacle and static obstacles for faster data collection.

        for(int i = 0; i < control_steps; i++) {
            traj->copy_onto_back(env.get_current_state());
            current_state = env.get_current_state();
            // assigns the control to control_point
            controller.compute_control(current_state, goal_state);
            control_traj->copy_onto_back(control_point);
            
            // Apply control to environment
            env.step(control_point, duration);
            num_steps += 1;

            if (!controller_params["allow_collision"].as<bool>() && env.is_in_collision()) {
                success = false;
                break;
            }
                
            if (env.is_in_goal_region(goal_state)) {
                success_count++;
                success = true;
                break;
            }
        }
        
        traj->copy_onto_back(env.get_current_state());
        // Save data only if flag is true
        if (save_data_flag) {
            save_data(data_path, traj, control_traj, params["xml_path"].as<std::string>(), 
                     goal_state, success, num_steps, env);
        }
        
        // After success is determined (after the control loop)
        std::vector<double> goal_pos;
        env.get_state_space()->copy_vector_from_point(goal_pos, goal_state);
        goal_results.push_back({goal_pos, success, num_steps});
        
    }

    
    // Extract environment config name from xml_path
    std::string xml_path = params["xml_path"].as<std::string>();
    size_t last_slash = xml_path.find_last_of("/");
    size_t first_slash = xml_path.find_first_of("/");
    std::string env_folder = xml_path.substr(first_slash + 1, last_slash - first_slash - 1);
    
    std::string env_config_name = xml_path.substr(last_slash + 1);
    env_config_name = env_config_name.substr(0, env_config_name.find_last_of("."));

    // Create aggregate data path and necessary folders
    std::string aggregate_data_path = params["aggregate_data_path"].as<std::string>();
    std::string final_folder = aggregate_data_path + "/" + env_folder;
    create_folder(aggregate_data_path);
    create_folder(aggregate_data_path + "/" + env_folder.substr(0, env_folder.find_first_of("/")));
    create_folder(final_folder);
    
    json aggregate_data;
    aggregate_data["environment"] = env.get_geom_info();
    aggregate_data["xml_path"] = params["xml_path"].as<std::string>();
    
    Eigen::MatrixXd positions(goal_results.size(), 2);
    Eigen::MatrixXd successes(goal_results.size(), 1);

    // Convert goal results to the desired format
    std::vector<std::vector<double>> goal_success_data;
    int k = 0;
    for (const auto& result : goal_results) {
        std::vector<double> entry = {
            result.position[0],  // x position
            result.position[1],  // y position
            result.success ? 1.0 : 0.0,  // success flag
            static_cast<double>(result.num_steps)  // number of steps taken
        };
        positions(k, 0) = result.position[0];
        positions(k, 1) = result.position[1];
        successes(k, 0) = result.success ? 1.0 : 0.0;

        k++;
        goal_success_data.push_back(entry);
    }

    aggregate_data["goal_success"] = goal_success_data;
    
    // compute radii of 100% success
    double radius_step = params["radius_step"].as<double>();
    double max_radius = 2.0 + radius_step;
    double reachability_radius = 0.0;
    // Calculate squared distances from origin for all points at once
    Eigen::VectorXd squared_distances = positions.rowwise().squaredNorm(); 
    for(double radius = radius_step; radius < max_radius; radius += radius_step) {
        double radius_squared = radius * radius;
        // Convert boolean comparison to double (0.0 or 1.0)
        Eigen::VectorXd within_radius = (squared_distances.array() <= radius_squared).cast<double>();
        double points_in_radius = within_radius.sum();
        
        if (points_in_radius > 0) {
            double success_rate = (within_radius.array() * successes.array()).sum() / points_in_radius;
            if (success_rate < 1.0) {
                break;
            }
        }
        reachability_radius = radius;
    }

    aggregate_data["reachability_radius"] = reachability_radius;

    // Add scene images to aggregate data
    create_scene_images(env.get_geom_info(), aggregate_data);

    // Save the aggregate data with the environment config name
    std::string aggregate_file = final_folder + "/" + env_config_name + ".json";
    std::cout << "Saving aggregate data to: " << aggregate_file << std::endl;
    std::ofstream file(aggregate_file);
    file << aggregate_data.dump(4);
    file.close();
    
    std::cout << "Success rate: " << success_count << "/" << total_goals << std::endl;


    return 0;
}