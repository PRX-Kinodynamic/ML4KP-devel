#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/rrt.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/planners/planner.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"

#include <fstream>

using namespace prx;

int main(int argc, char* argv[])
{
  auto params = param_loader("examples/basic/viz_robot.yaml", argc, argv);

  simulation_step = params["simulation_step"].as<double>();
  init_random(params["random_seed"].as<int>());

  std::string plant_name = params["/plant/name"].as<>();
  std::string plant_path = params["/plant/path"].as<>();
  auto plant = std::dynamic_pointer_cast<omnirobot_FO_t>(prx::system_factory_t::create_system(plant_name, plant_path));
  // auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
  prx_assert(plant != nullptr, "Plant is nullptr!");

  world_model_t world_model({ plant }, {});
  world_model.create_context("context", { plant_name }, {});
  auto context = world_model.get_context("context");

  // Two ways of accessing lengthy parameter paths
  int min_steps = params["plant"]["min_steps"].as<int>();
  int max_steps = params["/plant/max_steps"].as<int>();

  auto start_state = context.first->get_state_space()->make_point();
  auto goal_state = context.first->get_state_space()->make_point();

  auto lower_bounds = params["/plant/state_space_lower_bound"].as<std::vector<double>>();
  auto upper_bounds = params["/plant/state_space_upper_bound"].as<std::vector<double>>();
  context.first->get_state_space()->set_bounds(lower_bounds, upper_bounds);

  context.first->get_state_space()->copy_point_from_vector(start_state,
                                                           params["/plant/start_state"].as<std::vector<double>>());
  context.first->get_state_space()->copy_point_from_vector(goal_state,
                                                           params["/plant/goal_state"].as<std::vector<double>>());

  params.print();

  three_js_group_t* vis_group = new three_js_group_t({ plant }, {});

  std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();
  auto ss = context.system_group->get_state_space();
  auto cs = plant->get_control_space();

  // TODO: Add parameter space
  if (plant_name == "omnirobot_FO")
  {
    auto v_mu = plant->get_parameter<std::vector<double>>("mu");
    PRX_DEBUG_ITERABLE(v_mu)
    plant->set_parameter("mu", params["/plant/mu"].as<std::vector<double>>());
    v_mu = plant->get_parameter<std::vector<double>>("mu");
    PRX_DEBUG_ITERABLE(v_mu)
  }

  trajectory_t traj(ss);
  plan_t plan(cs);
  // auto ctrl =  plant -> get_control_space() -> make_point();
  // ss -> copy_from_point(start_state);
  // cs -> sample(ctrl);
  // cs -> copy_from_point(ctrl);
  PRX_DEBUG_PRINT

  plant->connect_points(start_state, goal_state, plan, traj);

  PRX_DEBUG_PRINT

  // for (int i = 0; i < 100; ++i)
  // {
  //     // std::cout << "state: " << ss -> print_memory(2) << std::endl;
  //     traj.copy_onto_back(ss);
  //     plant -> propagate(simulation_step);
  // }
  // void add_vis_infos(info_geometry_t info_type, const trajectory_t& traj, std::string body_name, space_t*
  // state_space, std::string color="0x000000");

  // vis_group -> add_vis_infos(info_geometry_t::LINE, traj, body_name, ss);

  vis_group->add_detailed_vis_infos(info_geometry_t::FULL_LINE, traj, body_name, ss);

  vis_group->add_animation(traj, ss, start_state);

  vis_group->output_html("viz_robot.html");

  delete vis_group;

  std::cout << "End of program" << std::endl;
}
