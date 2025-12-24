#include <fstream>
#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/aorrt.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/factor_graphs/plants/plants.hpp"
#include "prx/planning/planners/planner.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/planning/planner_functions/tree_fix_time_discretization.hpp"

int main(int argc, char* argv[])
{
  prx::param_loader params{};
  params["simulation_step"].set(0.1);
  params["random_seed"].set(112392);
  params["environment"].set("environments/empty.yaml");
  params.add_opts(argc, argv);

  if (params.exists("plant_file"))
  {
    params["plant"].add_file(params["plant_file"].as<>());
  }
  else
  {
    prx_throw("plant_file argument needed");
  }
  prx::simulation_step = params["simulation_step"].as<double>();
  prx::init_random(params["random_seed"].as<int>());

  auto obstacles = prx::load_obstacles(params["environment"].as<>());
  std::vector<std::shared_ptr<prx::movable_object_t>> obstacle_list = obstacles.second;
  std::vector<std::string> obstacle_names = obstacles.first;

  const std::string plant_name{ params["/plant/name"].as<>() };
  const std::string plant_path{ params["/plant/path"].as<>() };
  auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
  prx_assert(plant != nullptr, "Plant is nullptr!");

  plant->init(params["plant"]);

  prx::world_model_t world_model({ plant }, { obstacle_list });
  world_model.create_context("context", { plant_name }, { obstacle_names });
  auto context = world_model.get_context("context");
  std::shared_ptr<prx::system_group_t> sg{ prx::system_group(context) };
  std::shared_ptr<prx::collision_group_t> cg{ prx::collision_group(context) };

  prx::space_t* ss{ sg->get_state_space() };
  prx::space_t* cs{ sg->get_control_space() };

  prx::space_point_t start_state{ ss->make_point(params["start_state"].as<std::vector<double>>()) };
  prx::plan_t plan{ cs };
  prx::trajectory_t traj{ ss };

  plan.from_file(params["plan"].as<>());

  sg->propagate(start_state, plan, traj);

  prx::three_js_group_t* vis_group = new prx::three_js_group_t({ plant }, { obstacle_list });

  std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();

  // vis_group->add_vis_infos(info_geometry_t::LINE, aorrt_query.tree_visualization, body_name, ss);
  vis_group->add_detailed_vis_infos(prx::info_geometry_t::FULL_LINE, traj, body_name, ss);
  vis_group->add_animation(traj, ss, start_state);
  vis_group->output_html("open_loop.html");

  delete vis_group;

  return 0;
}
