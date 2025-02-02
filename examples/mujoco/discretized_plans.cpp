#include "prx/utilities/defs.hpp"
#include "prx/mujoco/mj_simulator.hpp"
#include "prx/planning/planners/dirt.hpp"
#include "prx/planning/tasks/navigate_task.hpp"
#include "prx/planning/tasks/manipulate_task.hpp"
#include <boost/filesystem.hpp>

#include <fstream>
#include <iostream>
#include <vector>
#include <random>
#include <nlohmann/json.hpp>

using namespace boost::filesystem;
using namespace prx;
using json = nlohmann::json;

// Randomly select obstacle and edge point
int uniform_random_int(int min, int max)
{
  double random_value = prx::uniform_random();  // Use prx::uniform_random to get a random double between 0 and 1
  return min + static_cast<int>(random_value * (max - min + 1));  // Scale and shift to the desired range
};

// Function to extract obstacle information
std::vector<std::tuple<std::vector<double>, double, std::vector<double>, std::string, bool,
                       std::vector<std::vector<double>>, std::string>>
read_obstacle_params(param_loader params)
{
  std::vector<std::tuple<std::vector<double>, double, std::vector<double>, std::string, bool,
                         std::vector<std::vector<double>>, std::string>>
      obstacles;

  // Access the obstacles from the parameters
  auto obstacles_params = params["worldbody"]["obstacles"];

  for (const auto& obstacle : obstacles_params)
  {
    std::vector<double> pos = obstacle["pos"].as<std::vector<double>>();
    double rotation = obstacle["rotation"].as<double>();
    std::vector<double> size = obstacle["size"].as<std::vector<double>>();
    std::string type = obstacle["type"].as<std::string>();
    bool movable = obstacle["movable"].as<bool>();
    std::string name = obstacle["name"].as<std::string>();  // Extract the name

    // Extract edge points if they exist
    std::vector<std::vector<double>> edge_points;
    if (obstacle["edge_points"])
    {
      edge_points = obstacle["edge_points"].as<std::vector<std::vector<double>>>();
    }

    // Store the obstacle information as a tuple (now including name)
    obstacles.emplace_back(pos, rotation, size, type, movable, edge_points, name);

    std::cout << "Obstacle: " << name << " (type: " << type << ", movable: " << movable << ")" << std::endl;
    std::cout << "Edge points: " << edge_points.size() << std::endl;
  }

  return obstacles;
}

void write_trees(dirt_query_t* dirt_query_ptr, std::string output_folder, std::string subfolder)
{
  std::string foldername = output_folder + subfolder + "/";

  if (!exists(foldername))
  {
    create_directories(foldername);
  }
  else
  {
    for (const auto& entry : directory_iterator(foldername))
    {
      remove(entry.path());
    }
  }
  unsigned count = 0;
  //
  for (auto traj : dirt_query_ptr->tree_visualization)
  {
    std::string filename = foldername + std::to_string(count) + ".txt";
    std::ofstream fout(filename);
    fout << traj.print(16);
    count++;
  }
}

void create_folder(std::string path)
{
  if (!exists(path))
  {
    create_directories(path);
  }
}

void call_planner(dirt_specification_t* spec, dirt_query_t* query, dirt_t* dirt, condition_check_t* checker)
{
  dirt->link_and_setup_spec(spec);
  dirt->preprocess();
  dirt->link_and_setup_query(query);

  dirt->resolve_query(checker);
  dirt->fulfill_query();
}

space_point_t navigate(param_loader params, simulation_context context, std::vector<double> goal_vec,
                       std::vector<double> goal_region_radius, dirt_query_t* dirt_query_ptr, dirt_t* dirt,
                       double* time_taken, plan_t* full_solution, std::string solution_folder,
                       std::vector<std::vector<double>>* all_stats, std::string task_name)
{
  navigate_task_t move_task = navigate_task_t(params, context, goal_vec);
  move_task.initialize(goal_region_radius);
  std::string solutions_path = solution_folder + "plans/";
  std::string trajectory_path = solution_folder + "trajectories/";
  std::string trees_path = solution_folder + "trees/";
  create_folder(solutions_path);
  create_folder(trees_path);
  create_folder(trajectory_path);

  dirt_query_ptr = move_task.get_query();

  condition_check_t* checker = move_task.get_condition_checker();
  dirt->reset();
  checker->reset();

  // // how long to run the planner
  // call_planner(move_task.get_specification(), dirt_query_ptr, dirt, checker);

  dirt->link_and_setup_spec(move_task.get_specification());
  dirt->preprocess();
  dirt->link_and_setup_query(dirt_query_ptr);

  for (int i = 0; i < 1; i++)
  {
    checker->reset();
    dirt->resolve_query(checker);
    dirt->fulfill_query();
  }

  *time_taken += dirt->current_solution_time;

  // write_trees(dirt_query_ptr, trees_path, task_name);

  plan_t solution = move_task.get_solution_plan();
  auto stats = dirt->get_statistics();

  if (solution.size() == 0)
  {
    auto condition_checker = params["condition_checker"];
    stats[0] = condition_checker["value"].as<double>();
  }
  all_stats->push_back(stats);

  solution.to_file(solutions_path + task_name + ".txt");
  dirt_query_ptr->solution_traj.to_file(trajectory_path + task_name + ".txt");
  *full_solution += solution;  // move_task.get_solution_plan();

  space_point_t new_start_state = dirt_query_ptr->solution_traj.back();

  dirt->reset();
  checker->reset();

  return new_start_state;
}

struct TrialData
{
  std::vector<double> goal;
  std::vector<double> goal_radius;
  std::string obstacle_name;
  int edge_point_index;
  bool success;
  double planning_time;
  std::vector<double> stats;
  std::vector<double> robot_position;
};

void save_trial_data(const std::string& output_folder, int trial_num, const TrialData& data)
{
  json j;
  j["trial_number"] = trial_num;
  j["goal"] = data.goal;
  j["goal_radius"] = data.goal_radius;
  j["obstacle_name"] = data.obstacle_name;
  j["edge_point_index"] = data.edge_point_index;
  j["success"] = data.success;
  j["planning_time"] = data.planning_time;
  j["stats"] = data.stats;
  j["robot_position"] = data.robot_position;

  std::string filename = output_folder + "trial_" + std::to_string(trial_num) + "_info.json";
  std::ofstream file(filename);
  file << j.dump(4);
  file.close();
}

int main()
{
  param_loader params;
  params = param_loader("examples/tasks/data.yaml");
  init_random(params["random_seed"].as<int>());

  // Setup environment
  std::string output_folder = params["output_folder"].as<std::string>();
  create_folder(output_folder);
  std::string output_folder_data = output_folder + "/data/";
  create_folder(output_folder_data);

  std::string env_name = params["env_name"].as<std::string>();
  std::string env_path = params["xml_path"].as<std::string>() + "/" + env_name + ".yaml";
  std::string xml_path = params["xml_path"].as<std::string>() + "/" + env_name + ".xml";

  // Load environment parameters and obstacles
  param_loader env_params;
  env_params = param_loader(env_path);
  auto obstacles = read_obstacle_params(env_params);

  int num_trials = params["num_trials"].as<int>();

  // Set the environment limits for navigation
  // params["navigate"]["env_xlim"] = std::vector<double>{ -env_size[0] / 2, env_size[0] / 2 };
  // params["navigate"]["env_ylim"] = std::vector<double>{ -env_size[1] / 2, env_size[1] / 2 };

  // Filter movable obstacles and select one randomly
  std::vector<std::tuple<std::vector<double>, double, std::vector<double>, std::string, bool,
                         std::vector<std::vector<double>>, std::string>>
      movable_obstacles;
  for (const auto& obstacle : obstacles)
  {
    auto [pos, rotation, size, type, movable, edge_points, name] = obstacle;
    if (movable)
    {
      movable_obstacles.push_back(obstacle);
    }
  }

  // if (movable_obstacles.empty())
  // {
  //   std::cerr << "No movable obstacles found!" << std::endl;
  //   return 1;
  // }

  // int obstacle_idx = uniform_random_int(0, movable_obstacles.size() - 1);
  // auto [obs_pos, obs_rotation, obs_size, obs_type, movable, edge_points, obs_name] = movable_obstacles[obstacle_idx];

  // if (edge_points.empty())
  // {
  //   std::cerr << "Selected obstacle has no edge points!" << std::endl;
  //   return 1;
  // }

  // int point_idx = uniform_random_int(0, edge_points.size() - 1);
  std::vector<double> selected_point = { 0.0, 0.0 };

  // Setup simulator
  bool visualize = params["visualize"].as<bool>();
  std::shared_ptr<prx::mujoco_simulator_t> sim = std::make_shared<prx::mujoco_simulator_t>(xml_path, visualize);
  sim->init_simulator();
  auto context = sim->get_context("mujoco");
  auto ss = context.first->get_state_space();
  auto cs = context.first->get_control_space();

  // auto ss = context.first
  // auto cs = contex

  // Get robot position from env params
  std::vector<double> robot_pos = env_params["worldbody"]["robot"]["pos"].as<std::vector<double>>();

  // Extract environment limits from env_params
  std::vector<double> env_size = env_params["env_size"].as<std::vector<double>>();
  std::cout << "Environment size: " << env_size[0] << ", " << env_size[1] << std::endl;

  // allowing the state space to be twice the size of the environment
  params["navigate"]["env_xlim"] = std::vector<double>{ -env_size[0], env_size[0] };
  params["navigate"]["env_ylim"] = std::vector<double>{ -env_size[1], env_size[1] };

  std::vector<double> true_goal_vec = { selected_point[0], selected_point[1], 0.0 };

  // Setup goal and planner
  std::vector<double> goal_vec = { selected_point[0] - robot_pos[0], selected_point[1] - robot_pos[1], 0.0 };
  std::vector<double> goal_region_radius = { 0.1, 0.1 };

  std::cout << "Goal: " << true_goal_vec[0] << ", " << true_goal_vec[1] << std::endl;

  // save the goal and goal region radius to a file
  std::ofstream goal_file(output_folder_data + "goal.txt");
  goal_file << true_goal_vec[0] << " " << true_goal_vec[1] << " " << goal_region_radius[0] << " "
            << goal_region_radius[1] << std::endl;
  goal_file.close();

  sim->set_goal(true_goal_vec);
  sim->set_goal_radius(0.1);

  // add collision pairs between robot and movable obstacles names
  // std::vector<std::string> obstacle_names;

  // Setup and run planner
  dirt_query_t* dirt_query_ptr;
  dirt_t dirt(params["planner_name"].as<std::string>());
  plan_t full_solution(context.first->get_control_space());
  std::vector<std::vector<double>> all_stats;

  // std::vector<double> initial_robot_pos;
  // initial_robot_pos.push_back(sim->d->qpos[0]);
  // initial_robot_pos.push_back(sim->d->qpos[1]);
  // std::cout << "number of geoms: " << sim->m->ngeom << std::endl;
  // for (int i = 0; i < sim->m->ngeom; i++)
  // {
  //   std::cout << "geom " << i << std::endl;
  // }

  int id = mj_name2id(sim->m, mjOBJ_GEOM, "robot");
  std::cout << "id: " << id << std::endl;
  mj_kinematics(sim->m, sim->d);  

  std::vector<double> robot_pos1 = { sim->d->geom_xpos[id * 3], sim->d->geom_xpos[id * 3 + 1],
                                     sim->d->geom_xpos[id * 3 + 2] };

  int success_count = 0;  // Add counter at the start of trials
  // discretize the goal position to cover the entire environment and run the planner for each goal position from a
  // fixed start position

  // Add discretization parameters
  double x_step = 0.1;  // Distance between points in x direction
  double y_step = 0.1;  // Distance between points in y direction

  // Calculate number of points in each dimension
  int x_points = static_cast<int>((env_size[0]) / x_step);
  int y_points = static_cast<int>((env_size[1] / 2) / y_step);

  // Fixed start position for the robot
  double start_x = robot_pos[0];  // Center of the environment
  double start_y = robot_pos[1];

  
  // Keep track of trial number
  int goal_num = 208;
  // Iterate through the grid
  for (int i = 13; i < x_points; i++)
  {
    for (int j = 0; j < y_points; j++)
    {
      goal_num++;
      int trial = 0;
      // for each goal position, run 10 trials
      for (int k = 0; k < 20; k++)
      {
        double time_taken = 0.0;
        trial++;
        std::cout << "\nStarting Trial " << trial << std::endl;

        sim->reset_simulation();

        // Add collision pairs
        for (const auto& obstacle : obstacles)
        {
          auto [pos, rotation, size, type, movable, edge_points, name] = obstacle;
          sim->add_pair({ "robot", name });
        }

        // Calculate goal position
        selected_point[0] = -env_size[0] / 2 + 0.125 + i * x_step;
        selected_point[1] = 0.0 + 0.125 + j * y_step;

        // Check if goal position is valid
        sim->d->qpos[0] = selected_point[0] - robot_pos1[0];
        sim->d->qpos[1] = selected_point[1] - robot_pos1[1];
        mj_kinematics(sim->m, sim->d);

        if (context.second->in_collision())
        {
          std::cout << "Goal position in collision, skipping" << std::endl;
          continue;
        }

        // Set robot to start position
        sim->d->qpos[0] = start_x - robot_pos1[0];
        sim->d->qpos[1] = start_y - robot_pos1[1];
        mj_kinematics(sim->m, sim->d);

        if (context.second->in_collision())
        {
          continue;
        }

        sim->reset_pairs();
        // Sample new obstacle and edge point for each trial
        // obstacle_idx = uniform_random_int(0, movable_obstacles.size() - 1);
        // auto [obs_pos, obs_rotation, obs_size, obs_type, movable, edge_points, obs_name] =
        // movable_obstacles[obstacle_idx];

        // if (edge_points.empty())
        // {
        //   std::cerr << "Selected obstacle has no edge points!" << std::endl;
        //   continue;  // Skip this trial and try the next one
        // }

        // point_idx = uniform_random_int(0, edge_points.size() - 1);
        // std::vector<double> selected_point = edge_points[point_idx];

        // Update goal vectors for new point
        std::vector<double> true_goal_vec = { selected_point[0], selected_point[1], 0.0 };

        std::vector<double> goal_vec = { selected_point[0] - robot_pos[0], selected_point[1] - robot_pos[1], 0.0 };

        std::cout << "Trial " << trial << " Goal: " << true_goal_vec[0] << ", " << true_goal_vec[1] << std::endl;
        sim->set_goal(true_goal_vec);

        // for (int k = 0; k < 5; k++){
        //     sim->step_simulation();
        // }
        // continue;

        // Get robot position after setting it
        std::vector<double> robot_position = { sim->d->geom_xpos[id * 3], sim->d->geom_xpos[id * 3 + 1],
                                               sim->d->geom_xpos[id * 3 + 2] };

        // Initialize trial data with new goal and robot position
        TrialData trial_data = { true_goal_vec, goal_region_radius, "", 0, false, 0.0, {}, robot_position };

        // Create trial-specific folders
        std::string trial_folder = output_folder_data + "goal_" + std::to_string(goal_num) + "/trial_" + std::to_string(trial) + "/";
        create_folder(trial_folder);

        // reset start state
        space_point_t new_start_state = ss->make_point();
        ss->copy_to(new_start_state);

        try
        {
          ss->copy_from(new_start_state);

          navigate(params["navigate"], context, goal_vec, goal_region_radius, dirt_query_ptr, &dirt, &time_taken,
                   &full_solution, trial_folder, &all_stats, "trial_" + std::to_string(trial));

          // Update trial data with success information
          trial_data.success = true;
          trial_data.planning_time = time_taken;
          if (!all_stats.empty())
          {
            trial_data.stats = all_stats[0];  // Store the first set of statistics
          }

          all_stats.clear();

          success_count++;  // Increment on successful planning
          std::cout << "Trial " << trial << " planning succeeded!" << std::endl;
        }
        catch (const std::exception& e)
        {
          // Update trial data with failure information
          trial_data.success = false;
          trial_data.planning_time = time_taken;
          if (!all_stats.empty())
          {
            trial_data.stats = {};
          }

          all_stats.clear();

          std::cerr << "Trial " << trial << " planning failed: " << e.what() << std::endl;
        }
        // Save trial information
        save_trial_data(trial_folder, trial, trial_data);
      }
    }
  }

  // Add summary at the end
  std::cout << "\nPlanning Summary: " << success_count << "/" << num_trials << " trials succeeded ("
            << (static_cast<double>(success_count) / num_trials * 100.0) << "% success rate)" << std::endl;

  return 0;
}