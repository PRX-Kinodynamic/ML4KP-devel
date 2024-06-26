#include "prx/utilities/defs.hpp"
#include "prx/mujoco/mj_simulator.hpp"
#include "prx/planning/planners/rrt.hpp"

#include <fstream>
#include <boost/filesystem.hpp>

using namespace boost::filesystem;
using namespace prx;

int main(int argc, char* argv[])
{
  param_loader params;
  params = param_loader("examples/JIST/visualize_plan.yaml");
  // if (argc < 2)
  // {
  //   params = param_loader("examples/JIST/visualize_plan.yaml");
  // }
  // else
  // {
  //   params = param_loader(argv[1]);
  // }
  init_random(params["random_seed"].as<int>());

  std::shared_ptr<prx::mujoco_simulator_t> sim =
      std::make_shared<prx::mujoco_simulator_t>(params["xml_path"].as<std::string>(), true);
  
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
  ss -> copy_to(start);
  // std::vector<double> start_vec = params["start_config"].as<std::vector<double>>();
  std::vector<double> goal_vec = params["goal_config"].as<std::vector<double>>();
  
  sim->set_goal(goal_vec);
  sim->set_goal_radius(params["goal_region_radius"].as<double>());

std::cout << "start" << start << std::endl;

  plan_t plan(cs);
  trajectory_t traj(ss);
  plan.from_file(params["plan_filename"].as<>());

  std::cout << "plan: " << plan << std::endl;

  sim->set_video_name(params["video_name"].as<>());

  std::cout << "video_name set" << std::endl;

  sim->set_record_video(params["record_video"].as<bool>());

  std::cout << "record_video set" << std::endl;

  context.first->propagate(start, plan, traj);

  std::cout << "plan propagated" << std::endl;

  sim->close_video();

  std::cout << "End of program!" << std::endl;
  
}