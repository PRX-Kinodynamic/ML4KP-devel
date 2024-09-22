#include "prx/planning/planners/rogue.hpp"

namespace prx
{
rogue_t::rogue_t(const std::string& new_name) : dirt_t(new_name)
{
  planner_name = "rogue";
}

rogue_t::~rogue_t()
{
  _reset();
}

void rogue_t::_link_and_setup_spec(planner_specification_t* spec)
{
  dirt_t::_link_and_setup_spec(spec);
  rogue_spec = dynamic_cast<rogue_specification_t*>(spec);
  prx_assert(rogue_spec != nullptr, "Rogue specification must be a rogue specification.");

  roadmap_heuristic = rogue_spec->roadmap_heuristic;
  node_expand = rogue_spec->node_expand;
  h = rogue_spec->h;

  prx_assert(!rogue_spec->use_pruning, "Rogue does not support pruning.");
}

bool rogue_t::_preprocess()
{
  tree.allocate_memory<rogue_node_t, rrt_edge_t>(1000);
  return true;
}

bool rogue_t::_link_and_setup_query(planner_query_t* query)
{
  rrt_query = dynamic_cast<rrt_query_t*>(query);
  prx_assert(rrt_query != nullptr, "RoGuE received an incorrect query type.");
  rogue_query = dynamic_cast<rogue_query_t*>(query);
  prx_assert(rogue_query != nullptr, "Rogue query must be a rogue query.");
  if (tree.num_vertices() == 0)
  {
    metric->clear();
    tree.clear();

    start_vertex = tree.add_vertex<rogue_node_t, rrt_edge_t>();
    goal_vertex = start_vertex;
    auto start_node = get_vertex(start_vertex);
    start_node->point = state_space->clone_point(rrt_query->start_state);
    start_node->cost_to_come = 0;
    start_node->dir_radius = 0;
    start_node->cost_to_go = h(start_node->point, rrt_query->goal_state);
    start_node->roadmap_cost_to_go = roadmap_heuristic(start_node->point);
    start_node->blossom_number = rogue_spec->blossom_number;
    start_node->local_goal_index = rogue_spec->start_node_local_goal;

    metric->add_node(start_node);
    previous_child = start_vertex;
    child_extension = true;
  }

  if (query->solution_plan.duration() > 0)
  {
    prx_warn("[RoGuE] Seeding the tree with the provided solution plan of duration: " << query->solution_plan.duration());
    std::pair<plan_t*, trajectory_t*> eg = std::make_pair(nullptr, nullptr);
    node_index_t current_node_idx = start_vertex;
    rogue_node_t* current_node = get_vertex(current_node_idx);

    for (unsigned i = 0; i < query->solution_plan.size(); i++)
    {
      // Get the first solution plan
      trajectory_t edge_traj(state_space);
      plan_t edge_plan(control_space);
      edge_plan.append_onto_back(query->solution_plan[i].duration);
      control_space->copy(edge_plan.back().control, query->solution_plan[i].control);
      propagate(current_node->point, edge_plan, edge_traj);

      // Check if the edge is valid
      bool valid = valid_check(edge_traj);
      if (!valid)
      {
        prx_warn("[RoGuE] Aborting seeding.");
        break;
      }

      // Add the edge to the tree
      eg = std::make_pair(new plan_t(edge_plan), new trajectory_t(edge_traj));
      double new_node_dir_radius = distance_function(edge_traj.back(), current_node->point);
      double edge_cost = cost_function(edge_traj, edge_plan);
      double end_heuristic = h(edge_traj.back(), rogue_query->goal_state);

      auto prox_nodes = metric->radius_and_closest_query(edge_traj.back(), std::max(new_node_dir_radius, max_radius));
      std::vector<rogue_node_t*> dir_updates;
      std::transform(prox_nodes.begin(), prox_nodes.end(), std::back_inserter(dir_updates),
                     [](proximity_node_t* prox_node) { return static_cast<rogue_node_t*>(prox_node); });
      std::for_each(dir_updates.begin(), dir_updates.end(), [&, this](rogue_node_t* node) {
        if (current_node->cost_to_come + edge_cost + end_heuristic > node->cost_to_come + node->cost_to_go)
        {
          new_node_dir_radius = std::min(new_node_dir_radius, distance_function(edge_traj.back(), node->point));
        }
      });

      add_edge_to_tree(eg, current_node, dir_updates, new_node_dir_radius);
      current_node_idx = tree.get_vertex_as<rogue_node_t>(current_node_idx)->get_children().front();
      current_node = get_vertex(current_node_idx);
    }
    prx_warn("[RoGuE] Seeding complete.");
  }

  timer.reset();
  iteration_count = 0;
  current_solution = 0;
  current_solution_iters = 0;
  current_solution_time = 0;
  return true;
}

void rogue_t::_resolve_query(condition_check_t* condition)
{
  do
  {
    if (!child_extension)
    {
      // sample state
      sample_state(sample_point);

      // find closest
      std::vector<rogue_node_t*> nodes;
      node_index_t closest_index;
      double best_distance = PRX_INFINITY;
      {
        auto prox_nodes = metric->radius_and_closest_query(sample_point, max_radius);
        std::transform(prox_nodes.begin(), prox_nodes.end(), std::back_inserter(nodes),
                       [](proximity_node_t* prox_node) { return static_cast<rogue_node_t*>(prox_node); });
        nodes.erase(std::remove_if(nodes.begin(), nodes.end(),
                                   [&, this](rogue_node_t* node) {
                                     const double test_dist = distance_function(sample_point, node->point);
                                     if (test_dist < best_distance)
                                     {
                                       best_distance = test_dist;
                                       closest_index = node->get_index();
                                     }
                                     if (test_dist <= node->dir_radius)
                                     {
                                       return false;
                                     }
                                     return true;
                                   }),
                    nodes.end());
      }
      if (nodes.size() == 0)
      {
        auto prox_nodes = metric->radius_and_closest_query(get_vertex(closest_index)->point, max_radius);
        std::transform(prox_nodes.begin(), prox_nodes.end(), std::back_inserter(nodes),
                       [](proximity_node_t* prox_node) { return static_cast<rogue_node_t*>(prox_node); });
        nodes.erase(std::remove_if(nodes.begin(), nodes.end(),
                                   [&, this](rogue_node_t* node) {
                                     const double test_dist =
                                         distance_function(get_vertex(closest_index)->point, node->point);
                                     if (test_dist <= node->dir_radius)
                                     {
                                       return false;
                                     }
                                     return true;
                                   }),
                    nodes.end());
      }
      // select one of the nodes at random
      //  prx_assert(nodes.size()>0,"Somehow the nearest nodes in DIRT is empty (should not happen).");
      if (nodes.size() > 0)
      {
        previous_child = nodes[uniform_int_random(0, nodes.size() - 1)]->get_index();
      }
      else
        previous_child = closest_index;
    }
    child_extension = false;

    auto closest_node = get_vertex(previous_child);

    std::vector<plan_t*> plans;
    std::vector<trajectory_t*> trajs;

    node_expand(closest_node, plans, trajs);
    closest_node->is_blossom_expand_done = true;

    for (int i = 0; i < plans.size(); i++)
    {
      closest_node->edge_generators.push_back(std::make_pair(plans[i], trajs[i]));
    }

    std::pair<plan_t*, trajectory_t*> eg = std::make_pair(nullptr, nullptr);
    double edge_cost;
    double end_heuristic;
    double new_node_dir_radius;
    std::vector<rogue_node_t*> dir_updates;

    eg = closest_node->edge_generators.back();
    closest_node->edge_generators.back() = std::make_pair(nullptr, nullptr);
    edge_cost = cost_function(*eg.second, *eg.first);
    end_heuristic = h(eg.second->back(), rogue_query->goal_state);

    // bnb
    if ((goal_vertex != start_vertex && closest_node->cost_to_come + edge_cost + end_heuristic > current_solution))
    {
      delete eg.first;
      delete eg.second;
      eg = std::make_pair(nullptr, nullptr);
      iteration_count++;
      continue;
    }

    double parent_distance = distance_function(eg.second->back(), closest_node->point);
    new_node_dir_radius = parent_distance;

    auto prox_nodes = metric->radius_and_closest_query(eg.second->back(), std::max(parent_distance, max_radius));
    dir_updates.clear();
    std::transform(prox_nodes.begin(), prox_nodes.end(), std::back_inserter(dir_updates),
                   [](proximity_node_t* prox_node) { return static_cast<rogue_node_t*>(prox_node); });
    std::for_each(dir_updates.begin(), dir_updates.end(), [&, this](rogue_node_t* node) {
      if (closest_node->cost_to_come + edge_cost + end_heuristic > node->cost_to_come + node->cost_to_go)
      {
        new_node_dir_radius = std::min(new_node_dir_radius, distance_function(eg.second->back(), node->point));
      }
    });

    // validity check
    bool valid = valid_check(*eg.second);

    if (!valid)
    {
      // std::cout << "Invalid edge!" << std::endl;
      delete eg.first;
      delete eg.second;
      eg = std::make_pair(nullptr, nullptr);
      iteration_count++;
      continue;
    }

    if (eg.first != nullptr)
    {
      add_edge_to_tree(eg, closest_node, dir_updates, new_node_dir_radius);
      delete eg.first;
      delete eg.second;
    }

    iteration_count++;
  } while (!condition->check());
  print_statistics();
}

void rogue_t::add_edge_to_tree(std::pair<plan_t*, trajectory_t*> eg, rogue_node_t* closest_node,
                               std::vector<rogue_node_t*> dir_updates, double new_node_dir_radius)
{
  auto node_index = tree.add_vertex<rogue_node_t, rrt_edge_t>();
  auto new_tree_node = tree.get_vertex_as<rogue_node_t>(node_index);
  new_tree_node->point = state_space->clone_point(eg.second->back());
  new_tree_node->bridge = true;
  edge_index_t edge_index = tree.add_edge(closest_node->get_index(), node_index);
  auto new_edge = tree.get_edge_as<rrt_edge_t>(edge_index);
  new_edge->plan = std::make_shared<plan_t>(*eg.first);
  new_edge->traj = std::make_shared<trajectory_t>(*eg.second);
  new_edge->edge_cost = cost_function(*eg.second, *eg.first);
  ;
  new_tree_node->cost_to_come = closest_node->cost_to_come + new_edge->edge_cost;
  new_tree_node->cost_to_go = h(eg.second->back(), rogue_query->goal_state);
  ;
  new_tree_node->roadmap_cost_to_go = roadmap_heuristic(eg.second->back());
  new_tree_node->blossom_number = rogue_spec->blossom_number;
  new_tree_node->dir_radius = new_node_dir_radius;

  new_tree_node->local_goal_index = closest_node->local_goal_index;

  max_radius = std::max(max_radius, new_node_dir_radius);
  // EXPERIMENTAL: Try commenting this line out. Behavior seems reasonable, but need to consider theoretical effects
  // get_vertex(start_vertex)->dir_radius = max_radius;

  std::for_each(dir_updates.begin(), dir_updates.end(), [&, this](rogue_node_t* node) {
    const double sibling_distance = distance_function(node->point, new_tree_node->point);
    if (new_tree_node->cost_to_come + new_tree_node->cost_to_go < node->cost_to_come + node->cost_to_go)
    {
      node->dir_radius = std::min(node->dir_radius, sibling_distance);
    }
  });
  if (new_tree_node->cost_to_go < closest_node->cost_to_go ||
      new_tree_node->roadmap_cost_to_go < closest_node->roadmap_cost_to_go)
  {
    child_extension = true;
  }
  previous_child = node_index;
  metric->add_node(new_tree_node.get());
  new_tree_node->bridge = false;
  update_goal(node_index);
}

void rogue_t::update_goal(node_index_t node_index)
{
  auto new_tree_node = tree.get_vertex_as<rogue_node_t>(node_index);
  if (rogue_query->goal_check(new_tree_node->point))
  {
    if (goal_vertex == start_vertex ||
        tree.get_vertex_as<rogue_node_t>(goal_vertex)->cost_to_come > new_tree_node->cost_to_come)
    {
      current_solution = new_tree_node->cost_to_come;
      current_solution_time = timer.measure();
      current_solution_iters = iteration_count;
      goal_vertex = node_index;
      std::cout << "[RoGuE] Found new goal: " << state_space->print_point(new_tree_node->point, 3);
      std::cout << " cost:" << new_tree_node->cost_to_come;
      std::cout << " time:" << current_solution_time;
      std::cout << " iter:" << current_solution_iters;
      std::cout << " nodes:" << metric->get_nr_nodes() << std::endl;
      bnb(start_vertex, current_solution);
      tree.remove_vertices();
    }
  }
}

void rogue_t::_reset()
{
  dirt_t::_reset();
}

void rogue_t::bnb(node_index_t v, double cost_bound, bool delete_flag)
{
  auto node = get_vertex(v);
  bool res = delete_flag || node->cost_to_come + node->cost_to_go > cost_bound;
  if (v == goal_vertex)
    res = false;
  std::list<node_index_t> children = node->get_children();
  for (auto child : children)
  {
    bnb(child, cost_bound, res);
  }
  if (res && is_leaf(v))
  {
    // remove the node that was previously there
    if (!node->bridge)
    {
      metric->remove_node(node);
      node->bridge = true;
    }
    for (int man_index = 0; man_index < node->edge_generators.size(); man_index++)
    {
      delete node->edge_generators[man_index].first;
      delete node->edge_generators[man_index].second;
    }
    node->edge_generators.clear();
    node->indices.clear();

    // remove the node
    //  tree.remove_vertex(v);
    auto node_ptr = tree.get_vertex_as<rogue_node_t>(v);
    tree.mark_vertex_for_removal(v);
  }
}

}  // namespace prx