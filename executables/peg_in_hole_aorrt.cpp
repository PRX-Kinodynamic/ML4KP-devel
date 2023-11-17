#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/aorrt.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/planners/planner.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"

#include <fstream>

int main(int argc, char* argv[])
{
  prx::param_loader params("executables/peg_in_hole.yaml", argc, argv);

  prx::simulation_step = params["simulation_step"].as<double>();
  prx::init_random(params["random_seed"].as<int>());

  auto obstacles = prx::load_obstacles(params["environment"].as<>());
  std::vector<std::shared_ptr<prx::movable_object_t>> obstacle_list = obstacles.second;
  std::vector<std::string> obstacle_names = obstacles.first;

  std::string plant_name = params["/plant/name"].as<>();
  std::string plant_path = params["/plant/path"].as<>();
  auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
  prx_assert(plant != nullptr, "Plant is nullptr!");

  prx::world_model_t world_model({ plant }, { obstacle_list });
  world_model.create_context("context", { plant_name }, { obstacle_names });
  auto context = world_model.get_context("context");
  std::shared_ptr<prx::system_group_t> sys_group{ context.first };

  prx::space_t* ss{ sys_group->get_state_space() };
  prx::space_t* cs{ sys_group->get_control_space() };
  prx::space_t* ps{ sys_group->get_parameter_space() };

  prx::aorrt_t aorrt(params["/planner/name"].as<>());
  prx::aorrt_specification_t aorrt_spec(context.first, context.second);

  // rrt_spec.valid_state = [](space_point_t& s)
  // {
  // Custom valid_state can be added here.
  // };

  // rrt_spec.valid_check = [&rrt_spec](trajectory_t& traj)
  // {
  // Custom valid_check goes here...
  // Basically for x in traj, call valid_state
  // };

  // rrt_spec.sample_plan = [&](plan_t& plan, space_point_t pose)
  // {
  // Add custom sample plan here
  // };

  aorrt_spec.distance_function = [&](const prx::space_point_t& s1, const prx::space_point_t& s2) {
    const double cost{ (Vec(s1).head(3) - Vec(s2).head(3)).squaredNorm() };
    return cost;
  };

  aorrt_spec.min_control_steps = params["plant/min_steps"].as<int>();
  aorrt_spec.max_control_steps = params["/plant/max_steps"].as<int>();

  prx::aorrt_query_t aorrt_query(context.first->get_state_space(), context.first->get_control_space());
  aorrt_query.start_state = context.first->get_state_space()->make_point();
  aorrt_query.goal_state = context.first->get_state_space()->make_point();

  const std::vector<double> ss_lower_bounds{ params["/plant/state_space/lower_bound"].as<std::vector<double>>() };
  const std::vector<double> ss_upper_bounds{ params["/plant/state_space/upper_bound"].as<std::vector<double>>() };

  const std::vector<double> cs_lower_bounds{ params["/plant/control_space/lower_bound"].as<std::vector<double>>() };
  const std::vector<double> cs_upper_bounds{ params["/plant/control_space/upper_bound"].as<std::vector<double>>() };

  const std::vector<double> ps_values{ params["/plant/parameter_space/values"].as<std::vector<double>>() };

  ss->set_bounds(ss_lower_bounds, ss_upper_bounds);
  cs->set_bounds(cs_lower_bounds, cs_upper_bounds);

  ps->copy_from(ps_values);

  ss->copy(aorrt_query.start_state, params["/plant/start_state"].as<std::vector<double>>());
  ss->copy(aorrt_query.goal_state, params["/plant/goal_state"].as<std::vector<double>>());

  aorrt_query.goal_region_radius = params["/planner/goal_region_radius"].as<double>();

  // Alternatively, change the goal_check function
  aorrt_query.goal_check = [&](prx::space_point_t pt)  // no-lint
  {
    const double dist_to_goal{ prx::space_t::euclidean_2d(pt, aorrt_query.goal_state, 0, 3) };
    return dist_to_goal < aorrt_query.goal_region_radius;
  };

  aorrt_query.get_visualization = params["visualize"].as<bool>();

  aorrt.link_and_setup_spec(&aorrt_spec);
  aorrt.preprocess();
  aorrt.link_and_setup_query(&aorrt_query);

  prx::condition_check_t checker(params["/planner/checker_type"].as<>(), params["/planner/checker_value"].as<int>());

  aorrt.resolve_query(&checker);
  aorrt.fulfill_query();

  // params.print();
  const std::string out_dir{ params["/out/dir"].as<>() };
  const std::string file_prefix{ params["/out/file_prefix"].as<>() };

  aorrt.to_files(file_prefix, out_dir);
  aorrt_query.solution_plan.to_file(out_dir + "/" + file_prefix + "_sln_plan.txt");
  aorrt_query.solution_traj.to_file(out_dir + "/" + file_prefix + "_sln_traj.txt");

  // aorrt_query.solution_traj.to_file();

  // Visualization
  prx::three_js_group_t* vis_group = new prx::three_js_group_t({ plant }, { obstacle_list });

  std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();

  vis_group->add_vis_infos(prx::info_geometry_t::LINE, aorrt_query.tree_visualization, body_name, ss);
  vis_group->add_detailed_vis_infos(prx::info_geometry_t::FULL_LINE, aorrt_query.solution_traj, body_name, ss);
  vis_group->add_vis_infos(prx::info_geometry_t::SPHERE, { Vec(aorrt_query.goal_state).head(3) }, "0xffff00",
                           aorrt_query.goal_region_radius);
  vis_group->add_animation(aorrt_query.solution_traj, ss, aorrt_query.start_state);
  vis_group->output_html("output.html");

  delete vis_group;

  std::cout << "End of program" << std::endl;
}
