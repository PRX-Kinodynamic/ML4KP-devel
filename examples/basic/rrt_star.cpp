#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/rrt_star.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/planners/planner.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"

#include <fstream>

using namespace prx;

int main(int argc, char* argv[])
{
  auto params = param_loader("examples/basic/rrt_star.yaml", argc, argv);

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
  const std::string context_name{ "rrt*_context" };
  world_model.create_context(context_name, { plant_name }, { obstacle_names });
  prx::world_model_context context{ world_model.get_context(context_name) };
  std::shared_ptr<prx::system_group_t> sys_group{ context.first };

  prx::space_t* ss{ sys_group->get_state_space() };
  prx::space_t* cs{ sys_group->get_control_space() };
  prx::space_t* ps{ sys_group->get_parameter_space() };
  rrt_star_t rrt_star("RRT*");
  rrt_star_specification_t rrt_star_spec(context.first, context.second);

  rrt_star_spec.eta_min = params["eta_min"].as<double>();
  rrt_star_spec.eta_max = params["eta_max"].as<double>();

  rrt_star_query_t rrt_star_query(ss, cs);
  rrt_star_query.start_state = ss->make_point();
  rrt_star_query.goal_state = ss->make_point();

  std::vector<double> lower_bounds{ params["/plant/state_space_lower_bound"].as<std::vector<double>>() };
  std::vector<double> upper_bounds{ params["/plant/state_space_upper_bound"].as<std::vector<double>>() };
  ss->set_bounds(lower_bounds, upper_bounds);

  ss->copy(rrt_star_query.start_state, params["/plant/start_state"].as<std::vector<double>>());
  ss->copy(rrt_star_query.goal_state, params["/plant/goal_state"].as<std::vector<double>>());

  rrt_star_query.get_visualization = params["visualize"].as<bool>();
  const std::string file_prefix{ "rrt_star" };
  const std::string out_dir{ prx::out_path };

  rrt_star.link_and_setup_spec(&rrt_star_spec);
  rrt_star.preprocess();
  rrt_star.link_and_setup_query(&rrt_star_query);

  if (params["grow_tree"].as<bool>())
  {
    prx::condition_check_t checker(params["/checker_type"].as<>(), params["/checker_value"].as<int>());

    rrt_star.resolve_query(&checker);
  }
  if (params["query_tree"].as<bool>())
  {
    rrt_star.from_files(file_prefix, out_dir);
    rrt_star.connect_goal();
  }
  rrt_star.fulfill_query();

  params.print();

  three_js_group_t* vis_group = new three_js_group_t({ plant }, { obstacle_list });

  std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();

  vis_group->add_vis_infos(info_geometry_t::LINE, rrt_star_query.tree_visualization, body_name, ss);
  vis_group->add_detailed_vis_infos(info_geometry_t::FULL_LINE, rrt_star_query.solution_traj, body_name, ss);
  vis_group->add_animation(rrt_star_query.solution_traj, ss, rrt_star_query.start_state);
  vis_group->output_html("rrt_star_output.html");

  delete vis_group;

  std::cout << "End of program" << std::endl;
}
