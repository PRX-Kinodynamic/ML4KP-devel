#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/aorrt.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/planning/planner_statistics.hpp"

#include <fstream>
#ifdef __cpp_lib_filesystem
#include <filesystem.hpp>
namespace fs = std::filesystem;
#else
#define _LIBCPP_NO_EXPERIMENTAL_DEPRECATION_WARNING_FILESYSTEM
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#endif

using namespace prx;

int main(int argc, char* argv[])
{
  param_loader params;
  if (argc > 1)
  {
    params = param_loader(argv[1]);
  }
  else
  {
    params = param_loader("examples/benchmark/run_dirt.yaml");
  }

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
  world_model.create_context("aorrt_context", { plant_name }, { obstacle_names });
  auto context = world_model.get_context("aorrt_context");

  aorrt_t aorrt(params["planner"].as<>());
  aorrt_specification_t aorrt_spec(context.first, context.second);

  aorrt_spec.distance_function = [&](const space_point_t& s1, const space_point_t& s2) {
    return space_t::euclidean_2d(s1, s2, 0, 3);
  };

  double maxvel = params["/plant/max_vel"].as<double>();
  assert(maxvel > 0);

  aorrt_spec.min_control_steps = params["/plant/min_steps"].as<int>();
  aorrt_spec.max_control_steps = params["/plant/max_steps"].as<int>();
  aorrt_spec.blossom_number = 1;

  aorrt_query_t aorrt_query(context.first->get_state_space(), context.first->get_control_space());
  aorrt_query.start_state = context.first->get_state_space()->make_point();
  aorrt_query.goal_state = context.first->get_state_space()->make_point();

  auto lower_bounds = params["/plant/state_space_lower_bound"].as<std::vector<double>>();
  auto upper_bounds = params["/plant/state_space_upper_bound"].as<std::vector<double>>();
  context.first->get_state_space()->set_bounds(lower_bounds, upper_bounds);

  context.first->get_state_space()->copy_point_from_vector(aorrt_query.start_state,
                                                           params["/plant/start_state"].as<std::vector<double>>());
  context.first->get_state_space()->copy_point_from_vector(aorrt_query.goal_state,
                                                           params["/plant/goal_state"].as<std::vector<double>>());

  aorrt_query.goal_region_radius = params["goal_region_radius"].as<double>();

  aorrt_query.goal_check = [&](const space_point_t& s) {
    return aorrt_spec.distance_function(s, aorrt_query.goal_state) < aorrt_query.goal_region_radius;
  };

  aorrt_query.get_visualization = params["visualize"].as<bool>();

  const int stats_runs = 10;
  condition_check_t checker("time", 1.0);
  const int num_calls = params["planning_time"].as<int>();

  if (!fs::exists(output_path + params["output_dir"].as<>()))
    fs::create_directory(output_path + params["output_dir"].as<>());

  for (int i = 0; i < stats_runs; i++)
  {
    aorrt.link_and_setup_spec(&aorrt_spec);
    aorrt.preprocess();
    aorrt.link_and_setup_query(&aorrt_query);

    planner_statistics_t stats;
    stats.link_planner(&aorrt);
    stats.link_criterion(&checker);
    stats.repeat_data_gathering(num_calls, false);

    std::string fname = output_path + params["output_dir"].as<>() + "/" + std::to_string(i) + ".txt";
    std::ofstream out(fname);
    out << stats.serialize();
    out.close();

    aorrt.reset();
    aorrt_query.clear_outputs();

    output_progress_bar(1.0 * i / stats_runs);
  }

  // three_js_group_t* vis_group = new three_js_group_t({plant},{obstacle_list});
  // std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();
  // auto ss = context.first -> get_state_space();
  // vis_group -> add_vis_infos(info_geometry_t::LINE, aorrt_query.tree_visualization, body_name, ss);
  // vis_group -> add_detailed_vis_infos(info_geometry_t::FULL_LINE, aorrt_query.solution_traj, body_name, ss);
  // vis_group -> add_animation(aorrt_query.solution_traj, ss, aorrt_query.start_state);
  // vis_group -> output_html("output.html");
  // delete vis_group;
}