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
                       plan_t* full_solution, std::string output_folder_trees, std::string task_name)
{
  navigate_task_t move_task = navigate_task_t(params, context, goal_vec);

  dirt_query_ptr = move_task.get_query();

  // // how long to run the planner
  call_planner(move_task.get_specification(), dirt_query_ptr, dirt, checker);
  *time_taken += dirt->current_solution_time;

  *full_solution += move_task.get_solution_plan();

  space_point_t new_start_state = dirt_query_ptr->solution_traj.back();
  write_trees(dirt_query_ptr, output_folder_trees, task_name);

  dirt->reset();
  checker->reset();

  return new_start_state;
}

int main(int argc, char* argv[])
{
  // load parameters from yaml
  param_loader params;z
  params = param_loader("examples/tasks/tasks.yaml");
  init_random(params["random_seed"].as<int>());

  int num_trials = params["num_trials"].as<int>();
  std::string output_folder = params["output_folder"].as<std::string>();
  create_folder(output_folder);
  std::string output_folder_trees = output_folder + "/trees/";
  create_folder(output_folder_trees);

  dirt_query_t* dirt_query_ptr;
  condition_check_t checker(params["checker_type"].as<>(), params["checker_value"].as<int>());

  std::vector<unsigned> pos_indices;
  std::vector<unsigned> angle_indices;

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

  // prx::constants::precision = 128;

  for (int i = 0; i < 5; i++)
  {
    sim->step_simulation();
  }

  dirt_t dirt(params["planner_name"].as<>());

  bool is_manipulate = params["do_manipulate"].as<bool>();

  goal_vec[0] = 1.2;
  int failures = 0;

  for (int i = 0; i < num_trials; i++)
  {
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

    if (is_manipulate)
    {
      space_point_t new_start_state;

      // manipulate task 1
      goal_vec.assign(n, 0.0);
      pos_indices = { 2, 3 };
      angle_indices = { 5, 6, 7, 8 };

      goal_vec[2] = 0.38;
      goal_vec[3] = 0.0;
      goal_vec[5] = 0.866025;  // 0.895;
      goal_vec[6] = 0.0;
      goal_vec[7] = 0.0;
      goal_vec[8] = -0.5;  // -0.446;

      try
      {
        new_start_state =
            manipulate(params["manipulate"], context, goal_vec, pos_indices, angle_indices, dirt_query_ptr, &dirt,
                       &checker, &time_taken, &full_solution, output_folder_trees, "task1");
        ss->copy_from(new_start_state);
      }
      catch (std::exception& e)
      {
        std::cout << e.what() << std::endl;
        failures += 1;
        continue;
      }

      // sim->add_pair({{"ball", "movable_cube1"}});

      // manipulate task 2
      goal_vec.assign(n, 0.0);
      goal_vec[0] = 1.0;

      pos_indices = { 9, 10 };
      angle_indices = { 12, 13, 14, 15 };

      goal_vec[9] = 0.68;
      goal_vec[10] = -0.1;
      goal_vec[12] = 1;
      goal_vec[13] = 0.0;
      goal_vec[14] = 0.0;
      goal_vec[15] = 0.0;

      try
      {
        new_start_state =
            manipulate(params["manipulate"], context, goal_vec, pos_indices, angle_indices, dirt_query_ptr, &dirt,
                       &checker, &time_taken, &full_solution, output_folder_trees, "task2");
        ss->copy_from(new_start_state);
      }
      catch (std::exception& e)
      {
        std::cout << e.what() << std::endl;
        failures += 1;
        continue;
      }

      // sim->add_pair({{"ball", "movable_cube1"}});

      // manipulate task 3
      goal_vec.assign(n, 0.0);
      goal_vec[0] = 1.0;

      pos_indices = { 16, 17 };
      angle_indices = { 19, 20, 21, 22 };

      goal_vec[16] = 1.0;
      goal_vec[17] = 0.00;
      goal_vec[19] = 1.0;
      goal_vec[20] = 0.0;
      goal_vec[21] = 0.0;
      goal_vec[22] = 0.0;

      try
      {
        new_start_state =
            manipulate(params["manipulate"], context, goal_vec, pos_indices, angle_indices, dirt_query_ptr, &dirt,
                       &checker, &time_taken, &full_solution, output_folder_trees, "task3");
        ss->copy_from(new_start_state);
      }
      catch (std::exception& e)
      {
        std::cout << e.what() << std::endl;
        failures += 1;
        continue;
      }
      // sim->add_pair({{"ball", "movable_cube3"}});
    }
    goal_vec.assign(n, 0.0);
    goal_vec[0] = 1.2;

    try
    {
      navigate(params["navigate"], context, goal_vec, dirt_query_ptr, &dirt, &checker, &time_taken, &full_solution,
               output_folder_trees, "task4");
    }
    catch (std::exception& e)
    {
      std::cout << e.what() << std::endl;
      failures += 1;
      continue;
    }

    record_time_taken.push_back(time_taken);

    if (params["output_plan"].as<bool>())
    {
      full_solution.to_file(output_folder + "/solution_" + std::to_string(i) + ".txt");
    }
  }

  std::cout << "average time taken: "
            << std::accumulate(record_time_taken.begin(), record_time_taken.end(), 0.0) / record_time_taken.size()
            << std::endl;
  std::cout << "success" << num_trials - failures << " out of " << num_trials << std::endl;

  std::cout << "End of program!" << std::endl;
}