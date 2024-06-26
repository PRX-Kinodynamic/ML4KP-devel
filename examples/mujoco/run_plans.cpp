#include "prx/utilities/defs.hpp"
#include "prx/mujoco/mj_simulator.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/planners/dirt.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/planning/planner_statistics.hpp"

// #ifdef __cpp_lib_filesystem
//     #include <boost/filesystem.hpp>
//     namespace fs = boost::filesystem;
// #else
//     #include <experimental/filesystem>
//     namespace fs = std::experimental::filesystem;
// #endif
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;
using namespace prx;

int main(int argc, char* argv[])
{
  param_loader params;
  params = param_loader("examples/JIST/plan_stats.yaml");
  init_random(params["random_seed"].as<int>());
  bool visualize = params["visualize"].as<bool>();

  std::shared_ptr<prx::mujoco_simulator_t> sim =
      std::make_shared<prx::mujoco_simulator_t>(params["xml_path"].as<std::string>(), visualize);
  sim->init_simulator();

  auto context = sim->get_context("mujoco");
  auto ss = context.first->get_state_space();
  auto cs = context.first->get_control_space();


  for (int i = 0; i < 100; i++)
  {
    sim->step_simulation();
  }

  dirt_t dirt(params["planner_name"].as<>());
  dirt_specification_t dirt_spec(context.first, context.second);

  float min_control_scaling = 0.1;
  float max_control_scaling = 1.0;
  dirt_spec.min_control_steps = min_control_scaling * (1.0 / simulation_step);
  dirt_spec.max_control_steps = max_control_scaling * (1.0 / simulation_step);

  dirt_spec.distance_function = [](const space_point_t& a, const space_point_t& b) {
    // std::cout << std::sqrt((b->at(0)-a->at(0))*(b->at(0)-a->at(0))+
    //                 (b->at(1)-a->at(1))*(b->at(1)-a->at(1))) << std::endl;

    return std::sqrt((b->at(0) - a->at(0)) * (b->at(0) - a->at(0)) + (b->at(1) - a->at(1)) * (b->at(1) - a->at(1)));
  };

  dirt_spec.blossom_number = params["blossom"].as<int>();

  // dirt query
  dirt_query_t dirt_query(context.first->get_state_space(), context.first->get_control_space());

  dirt_query.get_visualization = params["visualize"].as<bool>();

  dirt_query.goal_state = context.first->get_state_space()->make_point();

  dirt_query.start_state = context.first->get_state_space()->make_point();

  ss->copy_to(dirt_query.start_state);

  // goal region radius
  dirt_query.goal_region_radius = params["goal_region_radius"].as<double>();

  // goal
  std::vector<double> goal_vec = params["goal_config"].as<std::vector<double>>();
  for (int i = 0; i < 2; i++)
  {
    dirt_query.goal_state->at(i) = goal_vec[i];
  }

  // check whether sampled state is in goal region
  dirt_query.goal_check = [&](const space_point_t& point) {
    return dirt_spec.distance_function(point, dirt_query.goal_state) < dirt_query.goal_region_radius;
  };

  condition_check_t checker(params["checker_type"].as<>(), params["checker_value"].as<int>());  //

  int stats_runs = 3;
  std::ofstream fout;

  std::string output_dir = params["output_dir"].as<std::string>();
  std::string out_path = output_dir;
  if (!fs::exists(out_path))
  {
    fs::create_directory(out_path);
  }

  for (int i = 0; i < stats_runs; ++i)
  {
    
    dirt.link_and_setup_spec(&dirt_spec);
    dirt.preprocess();
    dirt.link_and_setup_query(&dirt_query);

    planner_statistics_t stats;
    stats.link_planner(&dirt);
    stats.link_criterion(&checker);
    simulation_time = 0.0;
    stats.repeat_data_gathering(10);
    simulation_time = 0.0;

    std::string full_name = out_path + params["planner_name"].as<std::string>() + "_" + std::to_string(i) + ".txt";
    fout.open(full_name);
    fout << stats.serialize() << std::endl;
    fout.close();
    

    // if (true)
    // {
    //   dirt.fulfill_query();
    //   std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();
    //   three_js_group_t* vis_group = new three_js_group_t({ plant }, { obstacle_list });
    //   vis_group->add_vis_infos(info_geometry_t::FULL_LINE, dirt_query.tree_visualization, body_name, ss, "0x000000");
    //   vis_group->output_html(params["output_dir"].as<std::string>() + params["planner_name"].as<std::string>() + "_" +
    //                          std::to_string(i) + ".html");
    //   delete vis_group;
    // }

    dirt.reset();
    dirt_query.clear_outputs();
    checker.reset();
    init_random(10);

    std::cout << "Finished run " << i << std::endl;
    
  }
}