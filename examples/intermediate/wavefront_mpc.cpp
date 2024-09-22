#include "prx/utilities/defs.hpp"
#include "prx/utilities/heuristics/heuristic_map.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/planning/planners/dirt.hpp"
#include "prx/visualization/three_js_group.hpp"


#include <fstream>

using namespace prx;

int main(int argc, char* argv[])
{
  std::string params_file;
  if (argc < 2)
  {
    params_file = "examples/intermediate/mushr_wavefront.yaml";
  }
  else
  {
    params_file = argv[1];
  }

  auto params = param_loader(params_file);
  simulation_step = params["simulation_step"].as<double>();
  init_random(params["random_seed"].as<int>());

  auto obstacles = load_obstacles(params["environment"].as<>());
  std::vector<std::shared_ptr<movable_object_t>> obstacle_list = obstacles.second;
  std::vector<std::string> obstacle_names = obstacles.first;

  std::string plant_name = params["/plant/name"].as<>();
  std::string plant_path = params["/plant/path"].as<>();
  auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
  prx_assert(plant != nullptr, "Plant is nullptr!");

  world_model_t world_model({ plant }, { obstacle_list });
  world_model.create_context("dirt_context", { plant_name }, { obstacle_names });

  auto context = world_model.get_context("dirt_context");
  auto sg = context.first;
  auto ss = sg->get_state_space();
  auto cs = sg->get_control_space();
  auto ps = sg->get_parameter_space();

  auto ss_lb = params["plant"]["state_space"]["lower_bound"].as<std::vector<double>>();
  auto ss_ub = params["plant"]["state_space"]["upper_bound"].as<std::vector<double>>();
  ss->set_bounds(ss_lb, ss_ub);
  auto cs_lb = params["plant"]["control_space"]["lower_bound"].as<std::vector<double>>();
  auto cs_ub = params["plant"]["control_space"]["upper_bound"].as<std::vector<double>>();
  cs->set_bounds(cs_lb, cs_ub);
  auto param_values = params["plant"]["parameter_space"]["values"].as<std::vector<double>>();\
  ps->copy_from(param_values);

  auto xlims = params["env_xlims"].as<std::vector<double>>();
  auto ylims = params["env_ylims"].as<std::vector<double>>();
  double xstep = params["env_xstep"].as<double>();
  double ystep = params["env_ystep"].as<double>();
  
  std::shared_ptr<dirt_specification_t> dirt_spec_ptr = std::make_shared<dirt_specification_t>(context.first, context.second);

  space_point_t start = ss -> make_point();
  auto start_vec = params["start"].as<std::vector<double>>();
  ss -> copy(start, start_vec);

  heuristic_map_t hmap(xlims[0], xlims[1], ylims[0], ylims[1], xstep, ystep, context);
  hmap.set_obstacle_grid();

  space_point_t goal = ss -> make_point();
  auto goal_vec = params["goal"].as<std::vector<double>>();
  ss -> copy(goal, goal_vec);
  hmap.set_heuristic_grid(goal);

  std::ofstream out_file(out_path + params["output_file"].as<std::string>());
  out_file << hmap;
  out_file.close();
  
  std::cout << "Heuristic value at start: " << hmap.get_cost(start) << std::endl;

  const double control_duration = params["control_duration"].as<double>();
  const int max_controls = params["max_controls"].as<int>();
  const unsigned max_steps = params["max_steps"].as<unsigned>();
  const double goal_radius = params["goal_radius"].as<double>();
  
  space_point_t current = ss -> clone_point(start);
  trajectory_t traj(ss), step_traj(ss);
  
  plan_t step_plan(cs);
  step_plan.append_onto_back(control_duration);

  std::vector<plan_t*> considered_plans;
  for (int i = 0; i < max_controls; i++)
  {
    considered_plans.push_back(new plan_t(cs));
    considered_plans[i] -> append_onto_back(control_duration);
  }

  for (int i = 0; i < max_steps; i++)
  {
    considered_plans.clear();
    double min_cost = PRX_INFINITY;
    for (int j = 0; j < max_controls; j++)
    {
      step_traj.clear();
      cs -> sample(considered_plans[j]->back().control);
      sg -> propagate(current, *considered_plans[j], step_traj);
      double end_cost = hmap.get_cost(step_traj.back());
      if (end_cost < min_cost)
      {
        min_cost = end_cost;
        cs -> copy(step_plan.back().control, considered_plans[j]->back().control);
      }
    }
    sg -> propagate(current, step_plan, step_traj);
    ss -> copy(current, step_traj.back());
    traj += step_traj;
    if (dirt_spec_ptr -> distance_function(current, goal) < goal_radius)
    {
      std::cout << "Reached goal!" << std::endl;
      break;
    }
  } 

  three_js_group_t* vis_group = new three_js_group_t({ plant }, { obstacle_list });
  std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();

  vis_group->add_detailed_vis_infos(info_geometry_t::FULL_LINE, traj, body_name, ss);
  vis_group->add_animation(traj, ss, start);
  vis_group->output_html("mpc.html");
  delete vis_group;
}
