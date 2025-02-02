// #ifndef TORCH_NOT_BUILT
#include "prx/utilities/defs.hpp"
#include "prx/mujoco/mj_simulator.hpp"
#include "prx/planning/planners/dirt.hpp"
#include "prx/planning/tasks/navigate_task.hpp"
#include "prx/planning/tasks/manipulate_task.hpp"
#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>

using namespace boost::filesystem;

using namespace prx;

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

space_point_t manipulate(param_loader params, simulation_context context, std::vector<double> goal_vec,
                         std::vector<unsigned> pos_indices, std::vector<unsigned> angle_indices,
                         dirt_query_t* dirt_query_ptr, dirt_t* dirt, condition_check_t* checker, double* time_taken,
                         plan_t* full_solution, std::string output_folder_trees, std::string task_name)
{
  manipulate_task_t push_task = manipulate_task_t(params, context, goal_vec, pos_indices, angle_indices);

  dirt_query_ptr = push_task.get_query();

  // // how long to run the planner
  call_planner(push_task.get_specification(), dirt_query_ptr, dirt, checker);
  *time_taken += dirt->current_solution_time;

  write_trees(dirt_query_ptr, output_folder_trees, task_name);
  *full_solution += push_task.get_solution_plan();
  space_point_t new_start_state = dirt_query_ptr->solution_traj.back();

  dirt->reset();
  checker->reset();

  return new_start_state;
}

space_point_t navigate(param_loader params, simulation_context context, std::vector<double> goal_vec,
                       std::vector<double> goal_region_radius, dirt_query_t* dirt_query_ptr, dirt_t* dirt, double* time_taken,
                       plan_t* full_solution, std::string solution_folder, std::vector<std::vector<double>>* all_stats,
                       std::string task_name)
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

  write_trees(dirt_query_ptr, trees_path, task_name);

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

std::vector<std::vector<double>> read_subgoals_from_file(std::string filename)
{
  std::ifstream file(filename);
  std::string line;
  std::vector<std::vector<double>> subgoals;

  while (std::getline(file, line))
  {
    std::istringstream iss(line);
    std::vector<double> subgoal;
    double value;
    while (iss >> value)
    {
      subgoal.push_back(value);
    }
    subgoals.push_back(subgoal);
  }
  return subgoals;
}

int main(int argc, char* argv[])
{
  // load parameters from yaml
  param_loader params;
  params = param_loader("examples/tasks/tasks.yaml");
  init_random(params["random_seed"].as<int>());

  int num_trials = params["num_trials"].as<int>();
  bool do_backtrack = params["backtrack"].as<bool>();
  bool do_retry = params["retry"].as<bool>();

  std::vector<double> goal_position = params["goal_position"].as<std::vector<double>>();
   
  std::string output_folder = params["output_folder"].as<std::string>();
  create_folder(output_folder);
  std::string output_folder_data = output_folder + "/data/";
  create_folder(output_folder_data);


  // intialize simulator with environment xml file
  bool visualize = params["visualize"].as<bool>();
  std::shared_ptr<prx::mujoco_simulator_t> sim =
      std::make_shared<prx::mujoco_simulator_t>(params["xml_path"].as<std::string>(), visualize);
  sim->init_simulator();

  auto context = sim->get_context("mujoco");
  auto ss = context.first->get_state_space();
  // auto sg = context.first;
  auto cs = context.first->get_control_space();
  int n = ss->get_dimension();

  std::vector<double> goal_vec(n, 0.0);
  // double goal_region_radius = 0.05;
   std::vector<double> goal_region_radius = {0., 0.};

  std::vector<std::vector<double>> subgoals;


  subgoals = read_subgoals_from_file(params["subgoals_file"].as<std::string>()); 

  // have been using subgoals as fixed goal regions for navigation task, change to subgoal regions instead and sample a state from the region to build a motion planning query


  plan_t full_solution(cs);
  double time_taken = 0.0;
  std::vector<double> record_time_taken;

  prx::constants::precision = 128;

  for (int i = 0; i < 5; i++)
  {
    sim->step_simulation();
  }

  dirt_query_t* dirt_query_ptr;
  dirt_t dirt(params["planner_name"].as<>());
  int failures = 0;
  std::unordered_map<int, int> backtracks;
  std::vector<int> task_failure_list;
  std::vector<std::vector<std::vector<double>>> full_stats;

  

  for (int i = 0; i < num_trials; i++)
  {
    // output_progress_bar(i * 1.0 / num_trials);
    std::vector<std::vector<double>> all_stats;

    std::cout << "solution_" << i << std::endl;
    
    backtracks[i] = 0;
    
    std::string solution_folder = output_folder_data + "trial_" + std::to_string(i) + "/";
    create_folder(solution_folder);

    sim->reset_simulation();

    for (int j = 0; j < 5; j++)
    {
      sim->step_simulation();
    }
    full_solution.clear();
    dirt.reset();
    time_taken = 0.0;

    // // // all 24 walls
    // sim->add_pair({ { "ball", "case" } });
    // sim->add_pair({ { "ball", "wall_1" },
    //                 { "ball", "wall_2" },
    //                 { "ball", "wall_3" },
    //                 { "ball", "wall_4" },
    //                 { "ball", "wall_5" },
    //                 { "ball", "wall_6" } });

    // sim->add_pair({ { "ball", "wall_7" },
    //                 { "ball", "wall_8" },
    //                 { "ball", "wall_9" },
    //                 { "ball", "wall_10" },
    //                 { "ball", "wall_11" },
    //                 { "ball", "wall_12" } });

    // sim->add_pair({ { "ball", "wall_13" },
    //                 { "ball", "wall_14" },
    //                 { "ball", "wall_15" },
    //                 { "ball", "wall_16" },
    //                 { "ball", "wall_17" },
    //                 { "ball", "wall_18" },
    //                 { "ball", "wall_19" },
    //                 { "ball", "wall_20" },
    //                 { "ball", "wall_21" },
    //                 { "ball", "wall_22" },
    //                 { "ball", "wall_23" },
    //                 { "ball", "wall_24" } });

    init_random(params["random_seed"].as<int>() + i);

    std::unordered_map<int, space_point_t> start_states;
    space_point_t new_start_state = ss->make_point();
    ss->copy_to(new_start_state);
    std::vector<int> repeats;
    repeats.assign(subgoals.size(), 0);
    // ss->copy_to_point(new_start_state);
    start_states[0] = new_start_state;
    // start_states.push_back(new_start_state);

    int ctr = 0;
    // std::cout << subgoals.size() << std::endl;
    // int random_skip = uniform_int_random(1, subgoals.size());
    bool too_many_repeats = false;
    bool task_failure = false;
    // 
    do
    {
      if (ctr < 0)
      {
        break;
      }
      std::vector<double> subgoal = subgoals[ctr];

      goal_vec.assign(n, 0.0);
      
      // Use subgoal as a region center
      double region_center_x = subgoal[0];
      double region_center_y = subgoal[1];
      double region_radius_x = subgoal[2];
      double region_radius_y = subgoal[3];

      // Sample a random goal state within the region
      goal_vec[0] = uniform_random(region_center_x - region_radius_x, region_center_x + region_radius_x);
      goal_vec[1] = uniform_random(region_center_y - region_radius_y, region_center_y + region_radius_y);

      // Set goal region radius to a small value for the sampled goal
      goal_region_radius[0] = 0.05;
      goal_region_radius[1] = 0.05;

      std::cout << "sampled goal: (" << goal_vec[0] << ", " << goal_vec[1] << ")" << std::endl;

      try 
      {
        // if repeat this subgoal more than 5 times, then call it a failure
        if (repeats[ctr] > 5)
        {
          too_many_repeats = true;
          throw std::runtime_error("repeated subgoal");
        }

        repeats[ctr] += 1;
        new_start_state = navigate(params["navigate"], context, goal_vec, goal_region_radius, dirt_query_ptr, &dirt,
                                   &time_taken, &full_solution, solution_folder, &all_stats,
                                   "subgoal" + std::to_string(ctr) + "_" + std::to_string(repeats[ctr]));

        ss->copy_from(new_start_state);
        ctr += 1;
        start_states[ctr] = new_start_state;
      }
      catch (std::exception& e)
      {
        std::cout << e.what() << std::endl;

        if (do_backtrack)
        {
          if ((ctr == 0) || too_many_repeats)
          {
            task_failure_list.push_back(i);
            task_failure = true;
            std::cout << "FAILED: solution_" << i << std::endl;
            failures += 1;
            break;
          }
          std::cout << "backtracking at " << ctr << " to " << ctr - 1 << std::endl;
          backtracks[i] += 1;
          time_taken += 30.0;
          ctr -= 1;
          std::cout << start_states[ctr]->at(0) << ", " << start_states[ctr]->at(1) << std::endl;
          ss->copy_from(start_states[ctr]);
        }
        else if (do_retry)  
        {

          if (too_many_repeats)
          {
            task_failure_list.push_back(i);
            task_failure = true;
            std::cout << "FAILED: solution_" << i << std::endl;
            failures += 1;
            break;
          }

          std::cout << "retrying at " << ctr << std::endl;
          repeats[ctr] += 1;
          std::cout << start_states[ctr]->at(0) << ", " << start_states[ctr]->at(1) << std::endl;
          ss->copy_from(start_states[ctr]);
        }
        else
        {
          task_failure_list.push_back(i);
          task_failure = true;
          failures += 1;
          std::cout << "FAILED: solution_" << i << std::endl;
          break;
        }
      }
    } while (ctr < subgoals.size());

    if (!task_failure)
    {
      record_time_taken.push_back(time_taken);
    }
    // std::cout << "time taken: " << time_taken << std::endl;

    if (params["output_plan"].as<bool>())
    {
      full_solution.to_file(output_folder + "/plan_trial_" + std::to_string(i) + ".txt");
    }

    double timer_measure = 0.0;

    for (auto stats : all_stats)
    {
      timer_measure += stats[0];
    }

    if (!task_failure){
      full_stats.push_back(all_stats);
    }
  }
  // output_progress_bar(num_trials * 1.0 / num_trials);

  std::cout << "Logging to file ..." << std::endl;

  std::ofstream fout(output_folder + "/time_taken.txt");
  for (auto time : record_time_taken)
  {
    fout << time << std::endl;
  }

  std::vector<double> cost_to_first_soln;
  std::vector<double> time_to_first_soln;
  std::vector<std::vector<double>> per_subgoal_time_taken;
  std::vector<std::vector<double>> per_subgoal_cost;
  std::vector<double> subgoal_time;
  subgoal_time.assign(subgoals.size(), 0.0);

  for (auto all_stats : full_stats)
  {
    double cost = 0.0;
    double time = 0.0;
    int ctr = 0;
    std::vector<double> individual_time_taken;
    std::vector<double> individual_cost;
    for (auto stats : all_stats)
    {
      cost += stats[3];
      time += stats[0];
      // individual_time_takestatsn.push_back(stats[0]);
      // individual_cost.push_back(stats[3]);
      // subgoal_time[ctr] += [0];
      ctr += 1;
    }
    time_to_first_soln.push_back(time);
    cost_to_first_soln.push_back(cost);
    // per_subgoal_time_taken.push_back(individual_time_taken);
    // per_subgoal_cost.push_back(individual_cost);
  }

  // std::ofstream fout1(output_folder + "/cost_to_first_soln.txt");
  // for (auto cost : cost_to_first_soln)
  // {
  //   fout1 << cost << std::endl;
  // }

  // std::ofstream fout2(output_folder + "/per_subgoal_time_taken.txt");
  // for (auto time : per_subgoal_time_taken)
  // {
  //   for (auto t : time)
  //   {
  //     fout2 << t << " ";
  //   }
  // fout2 << std::endl;
  // }

  // std::ofstream fout3(output_folder + "/per_subgoal_cost.txt");
  // for (auto cost : per_subgoal_cost)
  // {
  //   for (auto c : cost)
  //   {
  //     fout3 << c << " ";
  //   }
  //   fout3 << std::endl;
  // }

  std::ofstream fout4(output_folder + "/task_failure_list.txt");
  for (auto task : task_failure_list)
  {
    fout4 << task << std::endl;
  }

  if (do_backtrack)
  {
    std::ofstream fout_bt(output_folder + "/backtracks.txt");
    for (int i = 0; i < num_trials; i++)
    {
      fout_bt << backtracks[i] << std::endl;
    }
  }

  std::ofstream fout5(output_folder + "/final_statistics.txt");
  fout5 << "success: " << num_trials - failures << " out of " << num_trials << std::endl;
  fout5 << "average time taken: "
        << std::accumulate(time_to_first_soln.begin(), time_to_first_soln.end(), 0.0) / time_to_first_soln.size()
        << std::endl;
  fout5 << "average backtracks: "
        << std::accumulate(backtracks.begin(), backtracks.end(), 0.0,
                           [](double sum, const std::pair<int, int>& p) { return sum + p.second; }) /
               backtracks.size()
        << std::endl;

  // fout5 << "subgoal average times: ";
  // for (auto _time : subgoal_time)
  // {
  //   fout5 << _time / (num_trials - failures) << " ";
  // }
}