#ifndef TORCH_NOT_BUILT
#include "prx/utilities/defs.hpp"
#include "prx/planning/planners/dirt.hpp"
#include "prx/simulation/controllers/learned_controller.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/utilities/general/csv_reader.hpp"
#include "prx/utilities/data_structures/roadmap_with_gaps.hpp"
#include "prx/utilities/heuristics/heuristic_map.hpp"

#include <fstream>

using namespace prx;

int main(int argc, char* argv[])
{
  std::string params_file;
  if (argc < 2)
  {
    params_file = "examples/replanning/planning_simulation.yaml";
  }
  else
  {
    params_file = argv[1];
  }

  auto params = param_loader(params_file);
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
  world_model.create_context("dirt_context", { plant_name }, { obstacle_names });

  auto context = world_model.get_context("dirt_context");
  auto sg = context.first;
  auto ss = sg->get_state_space();
  auto cs = sg->get_control_space();
  auto ps = sg->get_parameter_space();

  auto ss_lb = params["plant"]["state_space"]["lower_bound"].as<std::vector<double>>();
  auto ss_ub = params["plant"]["state_space"]["upper_bound"].as<std::vector<double>>();
  ss->set_bounds(ss_lb, ss_ub);
  auto cs_lb = params["plant"]["control_space"]["lower_bound"].as<std::vector<double>>();
  auto cs_ub = params["plant"]["control_space"]["upper_bound"].as<std::vector<double>>();
  cs->set_bounds(cs_lb, cs_ub);
  auto param_values = params["plant"]["parameter_space"]["values"].as<std::vector<double>>();
  ps->copy_from(param_values);

  auto learned_controller_params = param_loader(params["learned_controller"].as<std::string>());
  learned_controller_t controller(plant, learned_controller_params);

  dirt_specification_t dirt_spec(context.first, context.second);

  auto xlims = params["/wavefront/xlims"].as<std::vector<double>>();
  auto ylims = params["/wavefront/ylims"].as<std::vector<double>>();
  double xstep = params["/wavefront/xstep"].as<double>();
  double ystep = params["/wavefront/ystep"].as<double>();
  heuristic_map_t hmap(xlims[0], xlims[1], ylims[0], ylims[1], xstep, ystep, context);
  hmap.set_obstacle_grid();

  dirt_spec.cost_function = [&hmap](const trajectory_t& traj, const plan_t& plan)
  {
    double accumulated_cost = 0;
    for(auto state : traj)
    {
      accumulated_cost += hmap.get_obstacle_cost(state);
    }
    return accumulated_cost;
  };

  dirt_query_t dirt_query(context.first->get_state_space(), context.first->get_control_space());
  dirt_query.goal_state = context.first->get_state_space()->make_point();
  dirt_query.start_state = context.first->get_state_space()->make_point();
  dirt_query.goal_region_radius = params["goal_region_radius"].as<double>();
  dirt_query.goal_check = [&](space_point_t point) {
    double diff2 = (point->at(0) - dirt_query.goal_state->at(0)) * (point->at(0) - dirt_query.goal_state->at(0)) +
                   (point->at(1) - dirt_query.goal_state->at(1)) * (point->at(1) - dirt_query.goal_state->at(1));
    if (learned_controller_params["goal_indices"].as<std::vector<int>>().size() > 2)
      diff2 += norm_angle_pi(point->at(2) - dirt_query.goal_state->at(2)) *
               norm_angle_pi(point->at(2) - dirt_query.goal_state->at(2));
    return std::sqrt(diff2) < dirt_query.goal_region_radius;
  };
  std::shared_ptr<roadmap_with_gaps_t> roadmap =
      std::make_shared<roadmap_with_gaps_t>(dirt_spec, dirt_query, controller);

  std::string landmarks_fname =
      lib_path + "resources/roadmaps/" + params["/roadmap/dir"].as<std::string>() + "landmarks.txt";
  std::cout << "Loading landmarks from " << landmarks_fname << std::endl;

  using prx::constants::separating_value;
  using prx::utilities::csv_reader_t;

  csv_reader_t reader(landmarks_fname, separating_value);
  using Line = std::vector<std::string>;
  while (reader.has_next_line())
  {
    Line line{ reader.next_line() };
    std::vector<double> landmark_vector;
    if (line.size() > 0)
    {
      for (auto& s : line)
      {
        landmark_vector.push_back(prx::utilities::convert_to<double>(s));
      }
      space_point_t landmark = ss->make_point();
      ss->copy_point_from_vector(landmark, landmark_vector);
      roadmap->add_landmark(landmark);
    }
  }

  roadmap->build_roadmap();

  std::string vertices_fname =
      lib_path + "resources/roadmaps/" + params["/roadmap/dir"].as<std::string>() + "vertices.txt";
  std::ofstream vertices_file;
  vertices_file.open(vertices_fname);
  auto vertices = roadmap->get_vertices();
  for (auto it : vertices)
  {
    vertices_file << it.first << ", " << ss->print_point(it.second->point) << std::endl;
  }
  vertices_file.close();

  std::string edges_fname = lib_path + "resources/roadmaps/" + params["/roadmap/dir"].as<std::string>() + "edges.txt";
  std::ofstream edges_file;
  edges_file.open(edges_fname);
  auto edges = roadmap->get_edges();
  for (auto e : edges)
  {
    edges_file << e.first << ", " << e.second << ", " << roadmap->get_edge_cost(e.first, e.second) << std::endl;
  }
  edges_file.close();

  std::ofstream traj_file;
  for (auto e : edges)
  {
    std::string traj_fname = lib_path + "resources/roadmaps/" + params["/roadmap/dir"].as<std::string>() + "traj_" +
                             std::to_string(e.first) + "_" + std::to_string(e.second) + ".txt";
    traj_file.open(traj_fname);
    dirt_query.clear_outputs();
    ss->copy_point(dirt_query.start_state, roadmap->get_vertex_point(e.first));
    ss->copy_point(dirt_query.goal_state, roadmap->get_vertex_point(e.second));
    controller.fulfill_query(dirt_spec, dirt_query);
    traj_file << dirt_query.solution_traj.print();
    traj_file.close();
  }
}
#else
int main(int argc, char* argv[])
{
  std::cout << "Torch not built!" << std::endl;
}
#endif