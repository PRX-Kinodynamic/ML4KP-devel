#include <fstream>
#include <iostream>
#include <string>
#include <utility>
#include <boost/algorithm/string/join.hpp>
#include <boost/optional.hpp>

#include "prx/utilities/defs.hpp"

#include "prx/simulation/plants/plants.hpp"
#include "prx/simulation/system_factory.hpp"
#include "prx/simulation/playback/trajectory.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"

#include "prx/planning/world_model.hpp"
#include "prx/simulation/system_group.hpp"
#include "prx/planning/planner_statistics.hpp"
#include "prx/planning/loaders/planner_loader.hpp"
#include "prx/visualization/three_js_group.hpp"

#include "prx/factor_graphs/defs.hpp"
#include "prx/factor_graphs/utilities/fg_logger.hpp"
#include "prx/factor_graphs/planning/trajectory_fg.hpp"
#include "prx/factor_graphs/planning/trajectory_optimizer.hpp"
#include "prx/factor_graphs/utilities/utilities_functions.hpp"
#include "prx/factor_graphs/planning/initialization_trajs_fg.hpp"

using namespace prx;

int main(int argc, char* argv[])
{
  auto params = param_loader("executables/factor_graphs/smoothing_trajs.yaml", argc, argv);

  simulation_step = params["simulation_step"].as<double>();

  std::string plant_name = params["/plant/name"].as<>();
  std::string plant_path = params["/plant/path"].as<>();

  auto obstacles = load_obstacles(params["environment"].as<>());

  auto obstacle_list = obstacles.second;
  auto obstacle_names = obstacles.first;

  auto plant = system_factory_t::create_system(plant_name, plant_path);
  world_model_t world_model({ plant }, {});
  world_model.create_context("context", { plant_name }, {});
  auto context = world_model.get_context("context");
  auto sg = context.first;
  const auto ss = sg->get_state_space();
  const auto cs = sg->get_control_space();
  auto ss_dim = ss->get_dimension();
  auto cs_dim = cs->get_dimension();

  auto cs_lb = params["/plant/control_space_lower_bound"].as<std::vector<double>>();
  auto cs_up = params["/plant/control_space_upper_bound"].as<std::vector<double>>();
  cs->set_bounds(cs_lb, cs_up);

  std::string file_prefix = out_path + "traj_opt_" + params["/plant/name"].as<>();
  std::string traj_in_name = params["traj_file_in"].as<>();

  trajectory_t traj_in(ss);
  trajectory_t traj_out(ss);
  trajectory_t traj_real(ss);
  traj_in.from_file(traj_in_name);

  auto start_state = ss->make_point();
  auto goal_state = ss->make_point();

  PRX_DEBUG_PRINT
  ss->copy_point(start_state, traj_in.front());
  ss->copy_point(goal_state, traj_in.back());

  std::cout << "[IN] traj front: " << traj_in.front() << std::endl;
  std::cout << "[IN] traj back: " << traj_in.back() << std::endl;

  plan_t plan_out(cs);

  trajectory_optimizer traj_opt(traj_in, plant, sg, plant_name);
  traj_opt.traj_opt_params.traj_rate = params["original_traj_rate"].as<double>();
  traj_opt.optimize(traj_out, plan_out);

  std::cout << "sln_plan:\n" << plan_out << std::endl;

  sg->propagate(start_state, plan_out, traj_real);

  std::cout << "[OUT] start_state: " << traj_out.front() << std::endl;
  std::cout << "[OUT] goal_state: " << traj_out.back() << std::endl;

  std::cout << "[REAL] start_state: " << traj_real.front() << std::endl;
  std::cout << "[REAL] goal_state: " << traj_real.back() << std::endl;

  traj_real.to_file(file_prefix + "_real.txt");

  three_js_group_t* vis_group = new three_js_group_t({ plant }, {});
  std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();

  vis_group->add_detailed_vis_infos(info_geometry_t::FULL_LINE, traj_in, body_name, ss);
  vis_group->add_animation(traj_in, ss, start_state);
  vis_group->output_html(params["/plant/name"].as<>() + "_traj_opt_in.html");

  vis_group->reset();
  vis_group->add_detailed_vis_infos(info_geometry_t::FULL_LINE, traj_out, body_name, ss);
  vis_group->add_animation(traj_out, ss, start_state);
  vis_group->output_html(params["/plant/name"].as<>() + "_traj_opt_out.html");

  vis_group->reset();
  vis_group->add_detailed_vis_infos(info_geometry_t::FULL_LINE, traj_real, body_name, ss);
  vis_group->add_animation(traj_real, ss, start_state);
  vis_group->output_html(params["/plant/name"].as<>() + "_traj_opt_real.html");

  delete vis_group;

  return 0;
}