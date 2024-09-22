#ifndef TORCH_NOT_BUILT
#include "prx/utilities/defs.hpp"
#include "prx/planning/planners/rogue.hpp"
#include "prx/simulation/controllers/learned_controller.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/planner_statistics.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/data_structures/roadmap_with_gaps.hpp"

#include <boost/filesystem.hpp>
using namespace boost::filesystem;

using namespace prx;

int main(int argc, char* argv[])
{
  param_loader params;
  if (argc < 2)
  {
    params = param_loader("examples/replanning/planning_simulation.yaml");
  }
  else
  {
    params = param_loader(argv[1]);
  }
  init_random(params["random_seed"].as<int>());
  simulation_step = params["simulation_step"].as<double>();

  auto obstacles = load_obstacles(params["environment"].as<>());
  std::vector<std::shared_ptr<movable_object_t>> obstacle_list = obstacles.second;
  std::vector<std::string> obstacle_names = obstacles.first;

  std::string plant_name = params["/plant/name"].as<>();
  std::string plant_path = params["/plant/path"].as<>();
  auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
  prx_assert(plant != nullptr, "Plant is nullptr!");

  world_model_t world_model({ plant }, { obstacle_list });
  world_model.create_context("rogue_context", { plant_name }, { obstacle_names });

  auto context = world_model.get_context("rogue_context");
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

  rogue_t dirt(params["planner_name"].as<>());

  rogue_specification_t rogue_spec(context.first, context.second);
  rogue_spec.blossom_number = 1;
  rogue_spec.use_pruning = false;

  rogue_spec.distance_function = [](const space_point_t& a, const space_point_t& b) {
    double diff = (a->at(0) - b->at(0)) * (a->at(0) - b->at(0)) + (a->at(1) - b->at(1)) * (a->at(1) - b->at(1));
    return sqrt(diff);
  };

  rogue_spec.h = [&](const prx::space_point_t& s, const prx::space_point_t& s2) {
    return rogue_spec.distance_function(s, s2) / 0.62;
  };

  distance_function_t goal_dist = [](const space_point_t& a, const space_point_t& b) {
    double diff2 = (a->at(0) - b->at(0)) * (a->at(0) - b->at(0)) + (a->at(1) - b->at(1)) * (a->at(1) - b->at(1));
    double angdiff2 = norm_angle_pi(a->at(2) - b->at(2)) *
                      norm_angle_pi(a->at(2) - b->at(2));
    diff2 += angdiff2;
    return sqrt(diff2);
  };

  rogue_spec.min_control_steps = 0.5 * (1.0 / simulation_step);
  rogue_spec.max_control_steps = 1.5 * (1.0 / simulation_step);

  rogue_query_t rogue_query(context.first->get_state_space(), context.first->get_control_space());
  rogue_query.goal_state = context.first->get_state_space()->make_point();
  rogue_query.start_state = context.first->get_state_space()->make_point();
  rogue_query.goal_region_radius = params["goal_region_radius"].as<double>();
  rogue_query.get_visualization = params["visualize"].as<bool>();

  std::vector<double> start = params["start_state"].as<std::vector<double>>();
  std::vector<double> goal = params["goal_state"].as<std::vector<double>>();

  for (int i = 0; i < start.size(); i++)
  {
    rogue_query.start_state->at(i) = start[i];
    rogue_query.goal_state->at(i) = goal[i];
  }

  rogue_query.goal_check = [&](const space_point_t& point) {
    return goal_dist(point, rogue_query.goal_state) < rogue_query.goal_region_radius;
  };

  rogue_query_t controller_query(context.first->get_state_space(), context.first->get_control_space());
  controller_query.start_state = ss->clone_point(rogue_query.start_state);
  controller_query.goal_state = ss->clone_point(rogue_query.goal_state);
  controller_query.goal_region_radius = rogue_query.goal_region_radius;
  controller_query.goal_check = [&](const space_point_t& point) {
    return goal_dist(point, controller_query.goal_state) < controller_query.goal_region_radius;
  };

  double duration = learned_controller_params["control_duration"].as<double>();
  std::shared_ptr<roadmap_with_gaps_t> roadmap = std::make_shared<roadmap_with_gaps_t>(rogue_spec, controller_query,controller);
  std::string vertices_fname =
      lib_path + "resources/roadmaps/" + params["/roadmap/dir"].as<std::string>() + "vertices.txt";
  std::string edges_fname =
      lib_path + "resources/roadmaps/" + params["/roadmap/dir"].as<std::string>() + "edges.txt";
  roadmap->load_roadmap_from_file(vertices_fname, edges_fname);

  std::cout << ss->print_point(rogue_query.start_state,4) << std::endl;
  std::cout << ss->print_point(rogue_query.goal_state,4) << std::endl;

  auto s_nn = roadmap->add_start(rogue_query.start_state);
  std::cout << "Added start node" << std::endl;
  auto g_nn = roadmap->add_goal(rogue_query.goal_state);
  std::cout << "Added goal node" << std::endl;

  graph_nearest_neighbors_t* metric = new graph_nearest_neighbors_t(goal_dist);
  auto vertices = roadmap->get_vertices();
  for (auto it : vertices)
  {
    metric->add_node(it.second.get());
  }

  roadmap->compute_wavefront(g_nn);

  std::vector<roadmap_with_gaps_node_t*> nodes;
  rogue_spec.roadmap_heuristic = [&](const space_point_t& s) { 
    nodes.clear();
    auto prox_nodes = metric->radius_and_closest_query(s, 0.25);
    
    std::transform(prox_nodes.begin(), prox_nodes.end(), std::back_inserter(nodes),
                   [](proximity_node_t* n) { return static_cast<roadmap_with_gaps_node_t*>(n); });

    double h = PRX_INFINITY;
    for (auto& node : nodes)
    {
      h = std::min(h, node->get_cost_to_go());
    }

    return h;
  };

  space_point_t local_goal = ss->make_point();
  rogue_spec.node_expand = [&](rogue_node_t* tree_node, std::vector<plan_t*>& plans, std::vector<trajectory_t*>& trajs)
  {
    if (!tree_node->is_blossom_expand_done)
    {
      nodes.clear();
      auto prox_nodes = metric->radius_query(tree_node->point, 0.25);

      std::transform(prox_nodes.begin(), prox_nodes.end(), std::back_inserter(nodes),
                     [](proximity_node_t* n) { return static_cast<roadmap_with_gaps_node_t*>(n); });

      auto nearest = nodes[0];
      int nearest_successor_index = nearest->get_successor_index();
      if (nearest_successor_index != -1)
      {
        ss -> copy_point(local_goal, roadmap->get_vertex_point(nearest_successor_index));
      }
      else
      {
        rogue_spec.sample_state(local_goal);
      }

      auto control = controller.get_control(tree_node->point, local_goal);

      trajectory_t traj(ss);
      plan_t plan(cs);

      traj.clear(); plan.clear();
      plan.append_onto_back(duration);
      cs -> copy_point_from_vector(plan.back().control, control);
      rogue_spec.propagate(tree_node->point, plan, traj);
      plans.push_back(new plan_t(plan));
      trajs.push_back(new trajectory_t(traj));
    }
    else
    {
      default_expand(tree_node->point, plans, trajs, 1, context.first, rogue_spec.sample_plan, rogue_spec.propagate);
    }
  };

  std::string out_dir = out_path + params["output_dir"].as<std::string>();
  if (!exists(out_dir))
  {
    create_directories(out_dir);
  }

  int stats_runs = 1;
  condition_check_t checker("time", 0.5);
  std::ofstream fout;
  for (int i = 0; i < stats_runs; i++)
  {
    init_random(params["random_seed"].as<int>() + i);
    dirt.link_and_setup_spec(&rogue_spec);
    dirt.preprocess();
    dirt.link_and_setup_query(&rogue_query);

    planner_statistics_t stats;
    stats.link_planner(&dirt);
    stats.link_criterion(&checker);
    stats.repeat_data_gathering(params["repeats"].as<int>());

    std::string full_name = out_dir + params["planner_name"].as<std::string>() + "_" + std::to_string(i) + ".txt";
    std::cout << "Writing to " << full_name << std::endl;
    fout.open(full_name);
    fout << stats.serialize() << std::endl;
    fout.close();

    if (params["visualize"].as<bool>())
    {
      dirt.fulfill_query();
      three_js_group_t* vis_group = new three_js_group_t({ plant }, { obstacle_list });
      std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();
      vis_group->add_vis_infos(info_geometry_t::LINE, rogue_query.tree_visualization, body_name, ss);
      vis_group->add_animation(rogue_query.solution_traj, ss, rogue_query.start_state);
      vis_group->add_detailed_vis_infos(info_geometry_t::FULL_LINE, rogue_query.solution_traj, body_name, ss);
      vis_group->output_html(params["planner_name"].as<std::string>() + "_" + std::to_string(i) + ".html");
      delete vis_group;
      // int count = 0;
      // for (auto traj : rogue_query.tree_visualization)
      // {
      //   std::string fname = out_path + "trajs/trajectory_" + std::to_string(count) + ".txt";
      //   fout.open(fname);
      //   fout << traj.print(4) << std::endl;
      //   fout.close();
      //   count++;
      // }
      std::string fname = out_path + "solution_"+std::to_string(i)+".txt";
      fout.open(fname);
      fout << rogue_query.solution_traj.print(4) << std::endl;
      fout.close();
    }

    rogue_query.clear_outputs();
    dirt.reset();
    checker.reset();
  }
}

#else
int main()
{
  std::cout << "Torch not built!" << std::endl;
}
#endif