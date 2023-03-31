#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/simulation/general/condition_check.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"

using namespace prx;

int main(int argc, char* argv[])
{
  auto params = param_loader("examples/basic/random_control.yaml", argc, argv);

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
  world_model.create_context("context", { plant_name }, { obstacle_names });
  auto context = world_model.get_context("context");
  std::shared_ptr<system_group_t> sg = world_model_t::get_system_group(context);

  const auto ss = sg->get_state_space();
  const auto cs = sg->get_control_space();

  auto start_state = ss->make_point();

  ss->copy(start_state, params["/plant/start_state"].as<std::vector<double>>());
  ss->copy_from(start_state);

  trajectory_t solution_traj(ss);

  plan_t plan(cs);
  for (int i = 0; i < 100; ++i)
  {
    plan.copy_onto_back(Eigen::Vector2d::Random(), prx::uniform_random(0.0, 1));
  }
  std::cout << plan << std::endl;
  sg->propagate(start_state, plan, solution_traj);

  three_js_group_t* vis_group = new three_js_group_t({ plant }, { obstacle_list });

  std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();

  vis_group->add_detailed_vis_infos(info_geometry_t::FULL_LINE, solution_traj, body_name, ss);

  vis_group->add_animation(solution_traj, ss, start_state);

  vis_group->output_html("random_control.html");

  delete vis_group;

  params.print();
}