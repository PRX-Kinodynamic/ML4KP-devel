#include "prx/utilities/defs.hpp"
#include "prx/mujoco/mj_simulator.hpp"
#include "prx/planning/planners/rrt.hpp"

#include <fstream>

using namespace prx;

int main(int argc, char* argv[])
{
  param_loader params;
  if (argc < 2)
  {
    params = param_loader("examples/JIST/franka.yaml");
  }
  else
  {
    params = param_loader(argv[1]);
  }
  init_random(params["random_seed"].as<int>());

  std::shared_ptr<prx::mujoco_simulator_t> sim =
      std::make_shared<prx::mujoco_simulator_t>(params["xml_path"].as<std::string>());
  
  sim->init_simulator();
  
  sim->set_cam_distance(params["cam_distance"].as<double>());
  sim->set_cam_elevation(params["cam_elevation"].as<double>());
  sim->set_cam_azimuth(params["cam_azimuth"].as<double>());
  

  auto context = sim->get_context("mujoco");
  auto sg = sim->get_context("mujoco");
  auto ss = context.first->get_state_space();
  auto cs = context.first->get_control_space();
  for (int i = 0; i < 100; i++)
  {
    sim->step_simulation();
  }

  auto start = ss->make_point();
  ss -> copy_to(start);
  std::vector<double> start_vec = params["start_config"].as<std::vector<double>>();
  std::vector<double> goal_vec = params["goal_config"].as<std::vector<double>>();

  /*
  double roll = 0.0, pitch = 0.0, yaw = 0.0;
  double z = 0.0;
  bool use_quadrotor = params["model_file"].as<std::string>().find("quadrotor") != std::string::npos;
  if (use_quadrotor)
  {
    z = start_vec[2];
  }
  else
  {
    roll = start_vec[5];
    z = start->at(2);
  }

  quaternion_t quat = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
                      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
  start -> at(0) = start_vec[0];
  start -> at(1) = start_vec[1];
  start -> at(2) = z;
  start -> at(3) = quat.w();
  start -> at(4) = quat.x();
  start -> at(5) = quat.y();
  start -> at(6) = quat.z();
  */
 
  plan_t plan(cs);
  trajectory_t traj(ss);
  plan.from_file(params["plan_filename"].as<>());
  // auto plan_from_file = plan.from_file(params["plan_filename"].as<>(), ",");
  /*
  for (auto line : plan)
  {
    plan.append_onto_back(line.back());
    line.pop_back();
    cs -> copy_point_from_vector(plan.back().control, line);
  }

  // plan.from_file(params["plan_filename"].as<>());
  sim->set_video_name(params["video_name"].as<>());

  sim->set_record_video(params["record_video"].as<bool>());
  context.first->propagate(start, plan, traj);

  sim->close_video();
  */
}