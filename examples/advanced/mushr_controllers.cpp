#include "prx/utilities/defs.hpp"
#include "prx/planning/planners/dirt.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/simulation/controllers/controllers.hpp"
#include "prx/visualization/three_js_group.hpp"

#include <boost/filesystem.hpp>
using namespace boost::filesystem;
using namespace prx;

int main(int argc, char* argv[])
{
  std::string params_file;
  if (argc < 2)
  {
    params_file = "examples/replanning/mushr_feedback.yaml";
  }
  else
  {
    params_file = argv[1];
  }

  auto params = param_loader(params_file);
  simulation_step = params["simulation_step"].as<double>();
  init_random(params["random_seed"].as<int>());

  std::string plant_name = params["/plant/name"].as<>();
  std::string plant_path = params["/plant/path"].as<>();
  auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
  prx_assert(plant != nullptr, "Plant is nullptr!");

  world_model_t world_model({ plant }, {});
  world_model.create_context("dirt_context", { plant_name }, {});

  auto context = world_model.get_context("dirt_context");
  auto sg = context.first;
  auto cg = context.second;
  auto ss = sg->get_state_space();
  auto cs = sg->get_control_space();
  auto ps = sg->get_parameter_space();

  std::vector<double> min_control_limits = params["/plant/control_space/lower_bound"].as<std::vector<double>>();
  std::vector<double> max_control_limits = params["/plant/control_space/upper_bound"].as<std::vector<double>>();
  std::vector<double> min_state_limits = params["/plant/state_space/lower_bound"].as<std::vector<double>>();
  std::vector<double> max_state_limits = params["/plant/state_space/upper_bound"].as<std::vector<double>>();
  ss->set_bounds(min_state_limits, max_state_limits);
  cs->set_bounds(min_control_limits, max_control_limits);
  std::vector<double> param_values = params["/plant/parameter_space/values"].as<std::vector<double>>();
  ps->copy_from(param_values);

  plan_t plan(cs);
  trajectory_t traj_to_track(ss);
  space_point_t start = ss->make_point();
  space_point_t goal = ss->make_point();

  std::vector<std::vector<double>> candidate_controls = { { -1., 1. }, { 0., 1. }, { 1., 1. } };

  for (int i = 0; i < 15; i++)
  {
    plan.append_onto_back(2.0);
    unsigned idx = uniform_int_random(0, candidate_controls.size());
    cs->copy_point_from_vector(plan.back().control, candidate_controls[idx]);
  }

  sg->propagate(start, plan, traj_to_track);
  ss->copy_point(goal, traj_to_track.back());

  param_loader controller_params(params["feedback_controller"].as<std::string>());
  feedback_controller_t<trajectory_t>* controller;

  if (controller_params["name"].as<std::string>() == "mushr_pure_pursuit")
  {
    controller = new mushr_pure_pursuit_t(plant, controller_params);
  }
  else if (controller_params["name"].as<std::string>() == "mushr_stanley")
  {
    controller = new mushr_stanley_t(plant, controller_params);
  }
  else
  {
    prx_throw("Unknown controller type: " << controller_params["name"].as<std::string>());
  }

  prx_assert(controller != nullptr, "Controller is nullptr!");

  controller->set_goal(goal);
  std::shared_ptr<trajectory_t> reference_traj = std::make_shared<trajectory_t>(traj_to_track);
  controller->set_points(reference_traj);

  trajectory_t controller_traj(ss);
  double control_duration = 1.0 / controller_params["frequency"].as<double>();
  std::cout << "Control duration: " << control_duration << std::endl;

  trajectory_t step_traj(ss);
  plan_t step_plan(cs);
  step_plan.append_onto_back(control_duration);
  space_point_t current = ss->make_point();
  ss->copy_point(current, start);

  double sim_time = 0;
  while (sim_time < 60.)
  {
    ss->copy_from(current);
    controller->compute_controls(step_plan.back().control);
    step_traj.clear();
    sg->propagate(current, step_plan, step_traj);
    controller_traj += step_traj;
    controller_traj.pop_back();
    current = step_traj.back();
    if (controller->goal_reached(current))
    {
      std::cout << "Goal reached!" << std::endl;
      controller_traj.copy_onto_back(current);
      break;
    }

    sim_time += control_duration;
  }

  std::cout << "Original trajectory duration: " << simulation_step * (traj_to_track.size() - 1) << std::endl;
  std::cout << "Controller trajectory duration: " << simulation_step * (controller_traj.size() - 1) << std::endl;

  three_js_group_t* vis_group = new three_js_group_t({ plant }, {});
  std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();
  vis_group->add_detailed_vis_infos(info_geometry_t::LINE, traj_to_track, body_name, ss);
  vis_group->add_detailed_vis_infos(info_geometry_t::FULL_LINE, controller_traj, body_name, ss);
  vis_group->add_animation(controller_traj, ss, start);
  vis_group->output_html("controller.html");

  delete vis_group;
}