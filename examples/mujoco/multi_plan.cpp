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


void call_planner(dirt_specification_t* spec, dirt_query_t* query, dirt_t* dirt, condition_check_t* checker)
{
  dirt->link_and_setup_spec(spec);
  dirt->preprocess();
  dirt->link_and_setup_query(query);
  dirt->resolve_query(checker);   
  dirt->fulfill_query();
}

space_point_t manipulate(param_loader params, simulation_context context, std::vector<double> goal_vec, std::vector<unsigned> pos_indices,  std::vector<unsigned> angle_indices, dirt_query_t* dirt_query_ptr, dirt_t* dirt, condition_check_t* checker, double* time_taken, plan_t* full_solution){
  
  manipulate_task_t push_task = manipulate_task_t(params, context, goal_vec, pos_indices, angle_indices);

  dirt_query_ptr = push_task.get_query();

  // // how long to run the planner
  call_planner(push_task.get_specification(), dirt_query_ptr, dirt, checker);
  *time_taken += dirt->current_solution_time;
  
  *full_solution += push_task.get_solution_plan();

  space_point_t new_start_state = dirt_query_ptr->solution_traj.back();

  dirt->reset();
  checker->reset();

  return new_start_state;
}


space_point_t navigate(param_loader params, simulation_context context, std::vector<double> goal_vec, dirt_query_t* dirt_query_ptr, dirt_t* dirt, condition_check_t* checker, double* time_taken, plan_t* full_solution){
  
  navigate_task_t move_task = navigate_task_t(params, context, goal_vec);

  dirt_query_ptr = move_task.get_query();

  // // how long to run the planner
  call_planner(move_task.get_specification(), dirt_query_ptr, dirt, checker);
  *time_taken += dirt->current_solution_time;
  
  *full_solution += move_task.get_solution_plan();

  space_point_t new_start_state = dirt_query_ptr->solution_traj.back();

  dirt->reset();
  checker->reset();

  return new_start_state;
}


int main(int argc, char* argv[])
{
  // load parameters from yaml
  param_loader params;
  params = param_loader("examples/tasks/tasks.yaml");
  init_random(params["random_seed"].as<int>());
 
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

  for (int i = 0; i < 5; i++)
  {
    sim->step_simulation();
  }

  dirt_t dirt(params["planner_name"].as<>());
  
  bool is_manipulate = params["do_manipulate"].as<bool>();

  goal_vec[0] = 1.0;

  if (is_manipulate){

    space_point_t new_start_state;

  
  // manipulate task 1
  goal_vec.assign(n, 0.0);
  pos_indices = { 2, 3 };
  angle_indices = { 5, 6, 7, 8 };
  
  goal_vec[2] = 0.384;
  goal_vec[3] = 0.001;
  goal_vec[5] = 0.895;
  goal_vec[6] = 0.0;
  goal_vec[7] = 0.0;
  goal_vec[8] = -0.446;

  new_start_state = manipulate(params["manipulate"], context, goal_vec, pos_indices, angle_indices, dirt_query_ptr, &dirt, &checker, &time_taken, &full_solution);
  ss->copy_from(new_start_state);

  // manipulate task 2
  goal_vec.assign(n, 0.0);
  goal_vec[0] = 1.0;
  
  pos_indices = { 9, 10 };
  angle_indices = { 12, 13, 14, 15 };

  goal_vec[9] = 0.569;
  goal_vec[10] = -0.058;
  goal_vec[12] = 0.960;
  goal_vec[13] = 0.0;
  goal_vec[14] = 0.0;
  goal_vec[15] = -0.280;

  new_start_state = manipulate(params["manipulate"], context, goal_vec, pos_indices, angle_indices, dirt_query_ptr, &dirt, &checker, &time_taken, &full_solution);
  ss->copy_from(new_start_state);

  // manipulate task 3
  goal_vec.assign(n, 0.0);
  goal_vec[0] = 1.0;

  pos_indices = { 16, 17 };
  angle_indices = { 19, 20, 21, 22 };

  goal_vec[16] = 0.850;
  goal_vec[17] = 0.054;
  goal_vec[19] = 0.950;
  goal_vec[20] = 0.0;
  goal_vec[21] = 0.0;
  goal_vec[22] = 0.311;

   new_start_state = manipulate(params["manipulate"], context, goal_vec, pos_indices, angle_indices, dirt_query_ptr, &dirt, &checker, &time_taken, &full_solution);
  ss->copy_from(new_start_state);

  goal_vec.assign(n, 0.0);
  goal_vec[0] = 1.0;

  navigate(params["navigate"], context, goal_vec, dirt_query_ptr, &dirt, &checker, &time_taken, &full_solution);
  
  prx::constants::precision = 32;
  if (params["output_plan"].as<bool>())
  {
    full_solution.to_file(params["output_plan_file"].as<std::string>());
  }

  std::cout << "time taken : " << time_taken <<  std::endl;
  std::cout << "End of program!" << std::endl;


  // navigate_task_t move_task2 = navigate_task_t(params["navigate"], context, goal_vec);
  // dirt_query_ptr = move_task2.get_query();

  // call_planner(move_task2.get_specification(), dirt_query_ptr, &dirt, &checker);
  // time_taken += dirt.current_solution_time;

  // full_solution += move_task2.get_solution_plan();

  // manipulate_task_t push_task1 = manipulate_task_t(params["manipulate"], context, goal_vec, pos_indices, angle_indices);
  // dirt_query_ptr = push_task1.get_query();

  // // // how long to run the planner
  // call_planner(push_task1.get_specification(), dirt_query_ptr, &dirt, &checker);
  // time_taken += dirt.current_solution_time;
  
  // full_solution += push_task1.get_solution_plan();


  // // set the ss to the last state of the solution trajectory
  // ss->copy_from(new_start_state);

  // // reset the planner
  // dirt.reset();
  // checker.reset();  // how long to run the planner

  //
  // goal_vec.assign(n, 0.0);
  // goal_vec[0] = 1.0;
  
  // pos_indices = { 9, 10 };
  // angle_indices = { 12, 13, 14, 15 };


  // // manipulate task
  // goal_vec[9] = 0.569;
  // goal_vec[10] = -0.058;
  // goal_vec[12] = 0.960;
  // goal_vec[13] = 0.0;
  // goal_vec[14] = 0.0;
  // goal_vec[15] = -0.280;


  // // manipulate(params["manipulate"], context, goal_vec, pos_indices, angle_indices, dirt_query_ptr, dirt, checker, &time_taken, &full_solution);


  // // 

  // manipulate_task_t push_task2 = manipulate_task_t(params["manipulate"], context, goal_vec, pos_indices, angle_indices);
  // dirt_query_ptr = push_task2.get_query();

  // // // how long to run the planner
  // call_planner(push_task2.get_specification(), dirt_query_ptr, &dirt, &checker);
  // time_taken += dirt.current_solution_time;
  
  // full_solution +=  push_task2.get_solution_plan();


  // // set the ss to the last state of the solution trajectory
  // ss->copy_from(dirt_query_ptr->solution_traj.back());

  // // // reset the planner
  // dirt.reset();
  // checker.reset();  // how long to run the planner
  
  // //

  // // 0.850,0.054,0.050,0.950,-0.000,-0.000,0.311

  // goal_vec.assign(n, 0.0);
  // goal_vec[0] = 1.0;

  // pos_indices = { 16, 17 };
  // angle_indices = { 19, 20, 21, 22 };

  // // manipulate task
  // goal_vec[16] = 0.850;
  // goal_vec[17] = 0.054;
  // goal_vec[19] = 0.950;
  // goal_vec[20] = 0.0;
  // goal_vec[21] = 0.0;
  // goal_vec[22] = 0.311;

  // // 

  // manipulate_task_t push_task3 = manipulate_task_t(params["manipulate"], context, goal_vec, pos_indices, angle_indices);
  // dirt_query_ptr = push_task3.get_query();

  // // // how long to run the planner
  // call_planner(push_task3.get_specification(), dirt_query_ptr, &dirt, &checker);
  // time_taken += dirt.current_solution_time;
  
  // full_solution += push_task3.get_solution_plan();


  // // set the ss to the last state of the solution trajectory
  // ss->copy_from(dirt_query_ptr->solution_traj.back());

  // // // reset the planner
  // dirt.reset();
  // checker.reset();  // how long to run the planner
  }

  // navigate_task

  // set goal here
  
}