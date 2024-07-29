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
  auto params = param_loader("examples/basic/plan_replay.yaml", argc, argv);

  simulation_step = params["simulation_step"].as<double>();

  auto obstacles = load_obstacles(params["environment"].as<>());
  std::vector<std::shared_ptr<movable_object_t>> obstacle_list = obstacles.second;
  std::vector<std::string> obstacle_names = obstacles.first;

  std::string plant_name = params["/plant/name"].as<>();
  std::string plant_path = params["/plant/path"].as<>();
  auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
  prx_assert(plant != nullptr, "Plant is nullptr!");

  world_model_t world_model({ plant }, { obstacle_list });
  world_model.create_context("context", { plant_name }, { obstacle_names });
  auto context = world_model.get_context("context");
  auto sg = context.system_group;

  space_t* ss{ context.first->get_state_space() };
  space_t* cs{ context.first->get_control_space() };

  prx::space_point_t start_state{ ss->make_point() };

  prx::plan_t plan(cs);
  prx::trajectory_t traj(ss);

  const std::string filename{ params["plan"].as<>() };

  plan.from_file(filename);
  sg->propagate(start_state, plan, traj);

  if (params["out/traj_to_file"].as<bool>())
  {
    traj.to_file(params["out/traj_file"].as<>());
  }

  auto lower_bounds = params["/plant/state_space_lower_bound"].as<std::vector<double>>();
  auto upper_bounds = params["/plant/state_space_upper_bound"].as<std::vector<double>>();
  ss->set_bounds(lower_bounds, upper_bounds);

  ss->copy(start_state, params["/plant/start_state"].as<std::vector<double>>());

  three_js_group_t* vis_group = new three_js_group_t({ plant }, { obstacle_list });

  std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();

  vis_group->add_detailed_vis_infos(info_geometry_t::FULL_LINE, traj, body_name, ss);

  vis_group->add_animation(traj, ss, start_state);

  vis_group->output_html("plan_replay.html");

  delete vis_group;

  std::cout << "End of program" << std::endl;
}
