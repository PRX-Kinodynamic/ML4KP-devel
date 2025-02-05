#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/progress_bar.hpp"
// #include "prx/mujoco/mj_simulator.hpp"
#include "push_environment.hpp"
#include "push_controller.hpp"

#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>

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
    // Create success/failure subfolder
    std::string folder = base_folder + "/" + (success ? "success" : "failure");
    create_folder(folder);

    // Count existing folders in success/failure directory
    int folder_count = count_folders(folder);
    std::string save_folder = folder + "/" + std::to_string(folder_count);
    create_folder(save_folder);

    // Save trajectories
    traj->to_file(save_folder + "/trajectory.txt");
    control_traj->to_file(save_folder + "/control_trajectory.txt");

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

int main(int argc, char* argv[])
{
    // Check if parameter file path is provided
    if (argc != 2) {
        std::cout << "Usage: " << argv[0] << " <parameter_file_path>" << std::endl;
        return 1;
    }

    // Load parameters from command line argument
    param_loader params(argv[1]); 
    init_random(params["random_seed"].as<int>());
    
    std::string data_path = params["data_path"].as<std::string>();
    create_folder(data_path);

    int folder_count = count_folders(data_path);
    double discretization = params["discretization"].as<double>();
    
    double duration = 0.1;
    int goal_count = 0;
    int success_count = 0;
    int num_steps = 0;

    // Initialize environment
    PushEnvironment env(params["xml_path"].as<std::string>(), 
                       params["visualize"].as<bool>());
    
    if(params["goal_sampling_mode"].as<std::string>() == "random") {
        env.set_goal_sampling_mode(PushEnvironment::GoalSamplingMode::RANDOM);
        env.set_random_goals(params["num_random_goals"].as<int>());
    }
    else if (params["goal_sampling_mode"].as<std::string>() == "discretized") {
        env.set_goal_sampling_mode(PushEnvironment::GoalSamplingMode::DISCRETIZED);
        env.set_discretization(discretization);
    }
    else {
        std::cout << "Invalid goal sampling mode" << std::endl;
        return 1;
    }

    // Initialize controller
    auto control_point = env.get_control_space_point(); // Get control point
    SimplePushController controller(control_point);
    
    int total_goals = env.get_total_goals();
    progress_bar_t progress(total_goals, "Processing goals");

    // Initialize for forward propagation and data collection
    space_point_t current_state = env.get_current_state();
    trajectory_t* traj = new trajectory_t(env.get_state_space());
    trajectory_t* control_traj = new trajectory_t(env.get_control_space());
    
    // Create success/failure folders at startup
    create_folder(data_path + "/success");
    create_folder(data_path + "/failure");

    // Iterate through all possible goals
    while (auto goal_state = env.next_goal()) {
        // Try to reach this goal

        // std::cout << "----------------------------------------" << std::endl;
        traj->clear();
        control_traj->clear();
        bool success = false;
        num_steps = 0;
        progress.update(++goal_count);

        env.add_pair(); // add collision pairs between the target movable obstacle and static obstacles for faster data collection.
        
        for(int i = 0; i < 500; i++) {
            traj->copy_onto_back(env.get_current_state());
            current_state = env.get_current_state();
            // assigns the control to control_point
            controller.compute_control(current_state, goal_state);
            control_traj->copy_onto_back(control_point);
            
            // Apply control to environment
            env.step(control_point, duration);
            num_steps += 1;

            if (env.is_in_collision()) {
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
        save_data(data_path, traj, control_traj, params["xml_path"].as<std::string>(), 
                  goal_state, success, num_steps, env);
    }

    std::cout << "Success rate: " << success_count << "/" << total_goals << std::endl;
    return 0;
}