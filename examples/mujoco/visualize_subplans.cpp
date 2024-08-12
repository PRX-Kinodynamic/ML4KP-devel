#include "prx/utilities/defs.hpp"
#include "prx/mujoco/mj_simulator.hpp"
#include "prx/planning/planners/rrt.hpp"

#include <fstream>
#include <boost/filesystem.hpp>

using namespace boost::filesystem;
using namespace prx;

void create_folder(std::string path)
{
  if (!exists(path))
  {
    create_directories(path);
  }
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
  param_loader params;
  params = param_loader("examples/JIST/visualize_plan.yaml");
  init_random(params["random_seed"].as<int>());

  std::shared_ptr<prx::mujoco_simulator_t> sim =
      std::make_shared<prx::mujoco_simulator_t>(params["xml_path"].as<std::string>(), true);

  prx::constants::precision = 128;
  sim->init_simulator();

  sim->set_cam_distance(params["cam_distance"].as<double>());
  sim->set_cam_elevation(params["cam_elevation"].as<double>());
  sim->set_cam_azimuth(params["cam_azimuth"].as<double>());
  

  sim->set_record_video(false);

  auto context = sim->get_context("mujoco");
  auto sg = sim->get_context("mujoco");
  auto ss = context.first->get_state_space();
  auto cs = context.first->get_control_space();
  for (int i = 0; i < 10; i++)
  {
    sim->step_simulation();
  }

  auto start = ss->make_point();
  ss->copy_to(start);
  // std::vector<double> start_vec = params["start_config"].as<std::vector<double>>();
  std::vector<double> goal_vec = params["goal_config"].as<std::vector<double>>();

  sim->set_goal(goal_vec);
  sim->set_goal_radius(params["goal_region_radius"].as<double>());

  std::cout << "start" << start << std::endl;

  plan_t plan(cs);
  trajectory_t traj(ss);

  std::vector<std::string> subplans;
  // loop through text files in the plan_filename directory

  

  std::string plan_folder = params["plan_folder"].as<std::string>();
  plan_folder += "/data";
  std::string video_file = "";
  video_file += plan_folder;
  std::string solution_id = "/" + params["solution_id"].as<std::string>();
  plan_folder += solution_id;
  video_file += solution_id;
  std::string solution_plans = "/solutions/";
  plan_folder += solution_plans;

  // video_file += "/videos";
  create_folder(video_file);
  video_file += "/video.mp4";


  // extract in sorted order solution plans from plan_folder (each file is .txt file in this directory, for ex. task1.txt, task2.txt, ...).
  // Make sure they are in order of task1.txt, task2.txt, ..
  int file_count = 1;
  for (directory_entry x : directory_iterator(plan_folder))
  {
    subplans.push_back(plan_folder+"task"+std::to_string(file_count)+".txt");
    file_count += 1;
  }

  for (auto plan: subplans)
  {
    std::cout << "plan: " << plan << std::endl;
  }
  
  sim->set_video_name(video_file);

  std::cout << "video_name set" << std::endl;

  sim->set_record_video(params["record_video"].as<bool>());

  std::cout << "record_video set" << std::endl;

  auto subgoals = read_subgoals_from_file(params["subgoals_filename"].as<std::string>());
  auto final_goal = params["final_goal"].as<std::vector<double>>();
  subgoals.push_back(final_goal);
  
  std::vector<double> cam_look_at = {0., 0., 0.};
  int ctr = 1;
  for (auto plans: subplans)
  {
    std::vector<double> subgoal_vec;
    subgoal_vec.push_back(subgoals[ctr][0]);
    subgoal_vec.push_back(subgoals[ctr][1]);
    subgoal_vec.push_back(0.);
    sim->set_goal(subgoal_vec);
    sim->set_goal_radius(subgoals[ctr][2]);
    // sim->

    plan.from_file(plans);
    std::cout << "plan: " << plan << std::endl;

    start = ss->make_point();
    ss->copy_to(start);
    // cam_look_at[0] = start->at(0);

    cam_look_at[0] = subgoal_vec[0];
    // cam_look_at[1] = start->at(1);

    context.first->propagate(start, plan, traj);
    sim->set_cam_lookat(cam_look_at);

    plan.clear();
    traj.clear();
    ctr += 1;
  }

  std::cout << "plan propagated" << std::endl;

  sim->close_video();

  std::cout << "End of program!" << std::endl;
}