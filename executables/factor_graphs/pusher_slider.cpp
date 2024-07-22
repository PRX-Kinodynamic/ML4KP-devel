#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>

#include <boost/algorithm/string/join.hpp>
#include <boost/optional.hpp>

#include "prx/simulation/plants/plants.hpp"
#include "prx/simulation/playback/trajectory.hpp"
#include "prx/simulation/system_factory.hpp"
#include "prx/simulation/system_group.hpp"
#include "prx/planning/world_model.hpp"

#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/csv_reader.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/visualization/three_js_group.hpp"

#include "prx/factor_graphs/lie_groups/se3.hpp"
#include "prx/factor_graphs/lie_groups/screw_axis.hpp"
#include "prx/factor_graphs/lie_groups/lie_integrator.hpp"
// #include "prx/factor_graphs/defs.hpp"
#include "prx/factor_graphs/utilities/symbols_factory.hpp"
#include "prx/factor_graphs/plants/pusher_slider.hpp"

#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/linear/JacobianFactor.h>

int main(int argc, char* argv[])
{
  // auto params = param_loader("examples/basic/rrt.yaml", argc, argv);

  // simulation_step = params["simulation_step"].as<double>();
  prx::simulation_step = 0.1;
  // init_random(params["random_seed"].as<int>());

  const std::string plant_name{ "pusher_slider" };
  auto plant = prx::system_factory_t::create_system(plant_name, plant_name);
  prx_assert(plant != nullptr, "Plant is nullptr!");

  prx::world_model_t world_model({ plant }, {});
  world_model.create_context("context", { plant_name }, {});
  auto context = world_model.get_context("context");
  std::shared_ptr<prx::system_group_t> sg{ prx::system_group(context) };

  prx::space_t* ss{ sg->get_state_space() };
  prx::space_t* cs{ sg->get_control_space() };
  prx::space_t* ps{ sg->get_parameter_space() };

  prx::plan_t plan(cs);
  prx::trajectory_t traj(ss);

  plan.copy_onto_back(Eigen::Vector2d(.5, 0.0), 1.0);
  plan.copy_onto_back(Eigen::Vector2d(0.5, 0.1), 1.0);
  plan.copy_onto_back(Eigen::Vector2d(0.5, 0.3), 1.0);

  ps->copy_from({ 1.87, 0.3 });

  prx::space_point_t x0{ ss->make_point() };
  ss->copy(x0, { 0.0, 0.0, 0.0, 0.50, 0.0 });
  sg->propagate(x0, plan, traj);

  const std::string body_name{ plant_name + "/body" };
  prx::three_js_group_t* vis_group = new prx::three_js_group_t({ plant }, {});
  vis_group->add_detailed_vis_infos(prx::info_geometry_t::FULL_LINE, traj, body_name, ss);
  vis_group->add_animation(traj, ss, x0);
  vis_group->output_html("pusher_slider.html");

  delete vis_group;
  return 0;
}
