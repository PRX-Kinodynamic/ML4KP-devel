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
  // PRX_DEBUG_PRINT
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
                       dirt_query_t* dirt_query_ptr, dirt_t* dirt, condition_check_t* checker, double* time_taken,
                       plan_t* full_solution, std::string output_folder_trees,
                       std::vector<std::vector<double>>* all_stats, std::string task_name)
{
  navigate_task_t move_task = navigate_task_t(params, context, goal_vec);
  // given a sequence of subgoal, how do you frame the kdp problem? 
  // here we are trying to come up with a sequence of kdp, 
  // tightness question -> how tight/precise does  
  // given a sequence of rooms/modes 
  // given an RL controller, if you understand it's reachability, you will understand how to use it.
  dirt_query_ptr = move_task.get_query();

  // // how long to run the planner
  call_planner(move_task.get_specification(), dirt_query_ptr, dirt, checker);

  
  *time_taken += dirt->current_solution_time;
  
  *full_solution += move_task.get_solution_plan();

  space_point_t new_start_state = dirt_query_ptr->solution_traj.back();
  write_trees(dirt_query_ptr, output_folder_trees, task_name);

  all_stats->push_back(dirt->get_statistics());

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
  std::string output_folder = params["output_folder"].as<std::string>();
  create_folder(output_folder);
  std::string output_folder_trees = output_folder + "/trees/";
  create_folder(output_folder_trees);

  dirt_query_t* dirt_query_ptr;

  std::vector<unsigned> pos_indices;
  std::vector<unsigned> angle_indices;

  std::vector<std::vector<double>> subgoals = read_subgoals_from_file(params["subgoals_file"].as<std::string>());

  double goal_region_radius = params["navigate"]["goal_pos_region_radius"].as<double>();

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
  plan_t full_solution(cs);
  double time_taken = 0.0;
  std::vector<double> record_time_taken;

  prx::constants::precision = 128;

  for (int i = 0; i < 5; i++)
  {
    sim->step_simulation();
  }

  dirt_t dirt(params["planner_name"].as<>());

  bool do_subgoals = params["do_subgoals"].as<bool>();

  std::string checker_type = params["checker_type"].as<std::string>();
  int checker_time = params["checker_value"].as<int>();

  if (!do_subgoals)
  {
    checker_time = subgoals.size() * checker_time;
  }

  int failures = 0;
  std::vector<int> task_failure;

  std::vector<std::vector<std::vector<double>>> full_stats;

  custom_check_t custom_checker = [ss, &goal_vec, goal_region_radius]() {
    auto pt1 = ss->make_point();
    ss->copy_to(pt1);

    double distance = std::sqrt((goal_vec[0] - pt1->at(0)) * (goal_vec[0] - pt1->at(0)) +
                                (goal_vec[1] - pt1->at(1)) * (goal_vec[1] - pt1->at(1)));

    return distance < goal_region_radius;
  };
  condition_check_t checker(checker_type, checker_time);
  condition_check_t checker2(custom_checker);

  checker.add_condition(&checker2);

  for (int i = 0; i < num_trials; i++)
  {
    output_progress_bar(i * 1.0 / num_trials);
    std::vector<std::vector<double>> all_stats;

      sim->reset_simulation();

    for (int j = 0; j < 5; j++)
    {
      sim->step_simulation();
    }
    full_solution.clear();
    dirt.reset();
    checker.reset();
    time_taken = 0.0;

    sim->add_pair({ { "ball", "case" } });
    init_random(params["random_seed"].as<int>() + i);

    space_point_t new_start_state;

    int ctr = -1;
    try
    {
      if (do_subgoals)
      {
        for (auto subgoal : subgoals)
        {
          ctr += 1;
          if (ctr == 0)
          {
            continue;
          }
          goal_vec.assign(n, 0.0);
          goal_vec[0] = subgoal[0];
          goal_vec[1] = subgoal[1];

          new_start_state =
              navigate(params["navigate"], context, goal_vec, dirt_query_ptr, &dirt, &checker, &time_taken,
                       &full_solution, output_folder_trees, &all_stats, "task" + std::to_string(ctr));
          ss->copy_from(new_start_state);
        }
      }

      ctr += 1;
      goal_vec.assign(n, 0.0);
      goal_vec[0] = 1.5;
      goal_vec[1] = 0.0;

      new_start_state = navigate(params["navigate"], context, goal_vec, dirt_query_ptr, &dirt, &checker, &time_taken,
                                 &full_solution, output_folder_trees, &all_stats, "task" + std::to_string(ctr));
      ss->copy_from(new_start_state);
    }
    catch (std::exception& e)
    {
      // std::cout << e.what() << std::endl;
      task_failure.push_back(ctr);
      failures += 1;
      continue;
    }

    record_time_taken.push_back(time_taken);

    if (params["output_plan"].as<bool>())
    {
      full_solution.to_file(output_folder + "/solution_" + std::to_string(i) + ".txt");
    }

    double timer_measure = 0.0;

    for (auto stats : all_stats)
    {
      timer_measure += stats[0];
    }

    full_stats.push_back(all_stats);
  }
  output_progress_bar(num_trials * 1.0 / num_trials);

  
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

  for(auto all_stats: full_stats){
    double cost = 0.0;
    double time = 0.0;
    int ctr = 0;
    std::vector<double> individual_time_taken;
    std::vector<double> individual_cost;
    for(auto stats: all_stats){
      cost += stats[3];
      time += stats[0];
      individual_time_taken.push_back(stats[0]);
      individual_cost.push_back(stats[3]);
      subgoal_time[ctr] += stats[0];
      ctr += 1;
    }
    time_to_first_soln.push_back(time);
    cost_to_first_soln.push_back(cost);
    per_subgoal_time_taken.push_back(individual_time_taken);
    per_subgoal_cost.push_back(individual_cost);
  }

  std::ofstream fout1(output_folder + "/cost_to_first_soln.txt");
  for (auto cost : cost_to_first_soln)
  {
    fout1 << cost << std::endl;
  }
  
  std::ofstream fout2(output_folder + "/per_subgoal_time_taken.txt");
  for (auto time: per_subgoal_time_taken)
  {
    for(auto t: time){
      fout2 << t << " ";
    }
    fout2 << std::endl;
  }

  std::ofstream fout3(output_folder + "/per_subgoal_cost.txt");
  for (auto cost: per_subgoal_cost)
  {
    for(auto c: cost){
      fout3 << c << " ";
    }
    fout3 << std::endl;
  }

  std::ofstream fout4(output_folder + "/task_failure.txt");
  for (auto task: task_failure)
  {
    fout4 << task << std::endl;
  }

  std::ofstream fout5(output_folder + "/final_statistics.txt");
  fout5 << "success: " << num_trials - failures << " out of " << num_trials << std::endl;
  fout5 << "average time taken: " << std::accumulate(time_to_first_soln.begin(), time_to_first_soln.end(), 0.0) / time_to_first_soln.size() << std::endl; 

  fout5 << "subgoal average times: ";
  for (auto _time: subgoal_time)
  {
    fout5 << _time/(num_trials-failures) << " ";
  }
}