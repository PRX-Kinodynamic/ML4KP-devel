#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/condition_check.hpp"
#include "prx/utilities/general/param_loader.hpp"

#include "prx/planning/world_model.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/simulation/plants/mushr.hpp"
#include "prx/utilities/general/csv_reader.hpp"

using namespace prx;

int main(int argc, char* argv[])
{
  simulation_step = 0.01;
  init_random(112392);
  prx::param_loader params("examples/basic/mushr_fg.yaml", argc, argv);

  const std::string plant_name{ "mushr" };
  const std::string plant_path{ "mushr" };
  auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
  prx_assert(plant != nullptr, "Plant is nullptr!");

  world_model_t world_model({ plant }, {});
  world_model.create_context("context", { plant_name }, {});
  auto context = world_model.get_context("context");

  auto system_group = context.first;
  prx::space_t* ss{ system_group->get_state_space() };
  prx::space_t* cs{ system_group->get_control_space() };
  prx::space_t* ps{ system_group->get_parameter_space() };

  auto start_state = ss->make_point();

  ss->copy(start_state, Eigen::Vector<double, 4>(1, 0, 1.57, 0));
  ps->copy_from(params["/plant/params"].as<std::vector<double>>());

  plan_t plan(cs);
  trajectory_t traj(ss);

  plan.copy_onto_back(Eigen::Vector2d(0, 0.5), 5);
  plan.copy_onto_back(Eigen::Vector2d(0.75, 0.5), 5);
  plan.copy_onto_back(Eigen::Vector2d(1.0, 0.0), 5);
  plan.copy_onto_back(Eigen::Vector2d(-1.0, 1.0), 5);
  plan.copy_onto_back(Eigen::Vector2d(0.0, 0.0), 1);
  plan.copy_onto_back(Eigen::Vector2d(1.0, -1.0), 10);
  plan.copy_onto_back(Eigen::Vector2d(0.0, 0.0), 1);

  system_group->propagate(start_state, plan, traj);

  traj.to_file(params["/out/trajectory_filename"].as<>());
  three_js_group_t* vis_group = new three_js_group_t({ plant }, {});

  std::string body_name = plant_name + "/body";

  vis_group->add_detailed_vis_infos(info_geometry_t::FULL_LINE, traj, body_name, ss);

  vis_group->add_animation(traj, ss, start_state);

  vis_group->output_html("mushr.html");

  delete vis_group;
}
