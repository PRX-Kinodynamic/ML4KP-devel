#include "prx/planning/planners/dirt_replanning.hpp"
#include <cstddef>
#include <memory>
#include "dirt_replanning.hpp"
#include "prx/utilities/data_structures/abstract_node.hpp"
// #include "prx/utilities/general/debug_utils.hpp"
#include "prx/utilities/general/random.hpp"
#include "prx/simulation/playback/trajectory.hpp"
#include "prx/utilities/spaces/space_snapshot.hpp"
namespace prx
{

dirt_replan_t::dirt_replan_t(const std::string& new_name)
  : rrt_t(new_name)
  // , _current_solution_type(dirt_replan_query_t::solution_type_t::NONE)
  , child_extension(true)
  , max_radius(0)
  , _resolve_profiler(nullptr)
{
  // , metric(nullptr)
}

dirt_replan_t::~dirt_replan_t()
{
  _reset();
}

void dirt_replan_t::_link_and_setup_spec(planner_specification_t* spec)
{
  rrt_t::_link_and_setup_spec(spec);
  // reset is always called before this
  dirt_spec = dynamic_cast<dirt_replan_specification_t*>(spec);
  prx_assert(dirt_spec != nullptr, "DIRT received an incorrect specification.");

  expand = dirt_spec->expand;
  _heuristic = dirt_spec->heuristic;
  _f_function = dirt_spec->f_function;
  // wavefront_h = dirt_spec->wavefront_h;
  contingency_check = dirt_spec->contingency_check;
  plan_safety_check = dirt_spec->plan_safety_check;

  planning_cycle_duration = dirt_spec->planning_cycle_duration;
  multiplier = 1.0 / simulation_step;

  if (dirt_spec->profile)
  {
    const std::string prefix{ dirt_spec->output_path + "/" + _planner_name };
    _resolve_profiler = std::make_shared<time_profiler_t>(prefix + "_resolve_query" + timestamp() + ".txt");
    _fulfill_profiler = std::make_shared<time_profiler_t>(prefix + "_fulfill_query" + timestamp() + ".txt");
  }
}
bool dirt_replan_t::_preprocess()
{
  tree().allocate_memory<dirt_replan_node_t, rrt_edge_t>(1000);
  return true;
}

bool dirt_replan_t::_link_and_setup_query(planner_query_t* query)
{
  rrt_query = dynamic_cast<rrt_query_t*>(query);
  prx_assert(rrt_query != nullptr, "DIRT received an incorrect query type.");
  dirt_replan_query = dynamic_cast<dirt_replan_query_t*>(query);
  prx_assert(dirt_replan_query != nullptr, "DIRT received an incorrect query type.");

  if (not dirt_spec->valid_state(rrt_query->start_state))
  {
    prx_warn("[dirt_replanning] Start state is not valid! " << (*(rrt_query->start_state)));
  }
  _current_solution_type = dirt_replan_query->sln_type;
  if (tree().num_vertices() == 0 ||
      !state_space->equal_points(tree().get_vertex_as<rrt_node_t>(start_vertex)->point, rrt_query->start_state))
  {
    // clear existing data structure
    metric->clear();
    tree().clear();

    start_vertex = tree().add_vertex<dirt_replan_node_t, rrt_edge_t>();
    goal_vertex = start_vertex;
    auto start_node = tree().get_vertex_as<dirt_replan_node_t>(start_vertex);
    // std::cout<<rrt_query->start_state<<std::endl;
    start_node->point = state_space->clone_point(rrt_query->start_state);
    start_node->cost_to_come = 0;
    start_node->dir_radius = 0;
    start_node->cost_to_go = _heuristic(start_node->point, dirt_replan_query->goal_state);
    start_node->blossom_number = dirt_spec->blossom_number;
    start_node->is_safe = false;
    metric->add_node(start_node.get());
    previous_child = start_vertex;
    _best_f_node = start_vertex;
    // _best_f_value = _heuristic(start_node->point, dirt_replan_query->goal_state);
    // _best_f_value = _f_function(0, start_node->cost_to_go);
    _best_f_value = std::numeric_limits<double>::infinity();  // <- necessary for the case "f=g"
    // best_cost = PRX_INFINITY;
    child_extension = true;

    _retainment_traj = std::make_shared<prx::trajectory_t>(state_space);
  }
  const double initial_f_value{ _best_f_value };
  // PRX_DBG_VARS(initial_f_value);
  // _best_f_node = start_vertex;
  // _best_f_value = _heuristic(start_node->point, dirt_replan_query->goal_state);

  _stats.reset();
  _timer.reset();
  _random_edges_counter.reset();
  _blossom_edges_counter.reset();
  if (dirt_replan_query->retainment)
  {
    const double retainment_duration{ dirt_replan_query->retained_plan.duration() };
    if (retainment_duration > prx::simulation_step)
    {
      PRX_MSG("[DIRT-Replanning] Using retainment!")
      space_point_t x0{ dirt_replan_query->start_state };
      // _retainment_traj->clear();
      // rrt_spec->_sg->propagate(x0, dirt_replan_query->retained_plan, *_retainment_traj);

      // const std::size_t total_states{ _retainment_traj->size() };
      auto current_node = tree().get_vertex_as<dirt_replan_node_t>(start_vertex);
      std::shared_ptr<plan_t> edge_plan{ std::make_shared<plan_t>(control_space) };
      std::shared_ptr<trajectory_t> edge_traj{ std::make_shared<trajectory_t>(state_space) };
      double new_node_dir_radius{ 0.0 };

      // dirt_replan_query->retained_plan.expand();
      // PRX_DBG_VARS(total_states, retainment_duration, dirt_replan_query->retained_plan.size())
      for (auto& step : dirt_replan_query->retained_plan)
      {
        edge_plan->clear();
        edge_traj->clear();

        edge_plan->copy_onto_back(step.control, step.duration);

        rrt_spec->_sg->propagate(x0, *edge_plan, *edge_traj);

        if (valid_check(*edge_traj))
        {
          new_node_dir_radius = distance_function(edge_traj->back(), current_node->point);

          std::pair<plan_t*, trajectory_t*> pair = { edge_plan.get(), edge_traj.get() };
          node_index_t new_node_idx{ add_edge_to_tree(pair, current_node.get(), {}, new_node_dir_radius) };

          current_node = tree().get_vertex_as<dirt_replan_node_t>(new_node_idx);

          update_goal(new_node_idx);
          x0 = edge_traj->back();
          // PRX_DBG_VARS(_best_f_node, _best_f_value);
        }
        else
        {
          PRX_MSG("[DIRT-Replanning] Retainment collided!")

          break;  // trajectory is in collision -> stop
        }
      }

      PRX_MSG("[DIRT-Replanning] Retainment done!")
    }
  }

  return true;
}

void dirt_replan_t::_resolve_query(condition_check_t* condition)
{
  // run for a certain amount of time
  do
  {
    prx::time_profiler_t::reset(_resolve_profiler);

    if (not child_extension)
    {
      // Regular propagation
      // sample state
      sample_state(sample_point);

      // find closest
      std::vector<dirt_replan_node_t*> nodes;
      node_index_t closest_index;
      double best_distance = PRX_INFINITY;
      {
        auto prox_nodes = metric->radius_and_closest_query(sample_point, max_radius);
        std::transform(prox_nodes.begin(), prox_nodes.end(), std::back_inserter(nodes),
                       [](proximity_node_t* prox_node) { return static_cast<dirt_replan_node_t*>(prox_node); });
        nodes.erase(std::remove_if(nodes.begin(), nodes.end(),
                                   [&, this](dirt_replan_node_t* node) {
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
                       [](proximity_node_t* prox_node) { return static_cast<dirt_replan_node_t*>(prox_node); });
        nodes.erase(std::remove_if(nodes.begin(), nodes.end(),
                                   [&, this](dirt_replan_node_t* node) {
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
      // std::cout << "Node selected for expansion: " << state_space->print_point(get_vertex(previous_child)->point) <<
      // std::endl;
    }

    prx::time_profiler_t::measure(_resolve_profiler);  // no child extension

    child_extension = false;

    auto closest_node = get_vertex(previous_child);

    bool is_blossom_expand = false;
    // if no maneuvers, get some
    if (closest_node->indices.size() == 0)
    {
      closest_node->edge_generators.clear();
      std::vector<plan_t*> plans;
      std::vector<trajectory_t*> trajs;
      expand(closest_node->point, plans, trajs, closest_node->blossom_number, !(closest_node->is_blossom_expand_done));

      if (closest_node->is_blossom_expand_done)
        closest_node->random_expand = true;
      else
        closest_node->is_blossom_expand_done = true;

      closest_node->blossom_number = 1;
      for (int i = 0; i < plans.size(); i++)
      {
        closest_node->edge_generators.push_back(std::make_pair(plans[i], trajs[i]));
      }

      std::vector<double> pred_values;
      int index = 0;
      for (auto& temp_eg : closest_node->edge_generators)
      {
        pred_values.push_back(_heuristic(temp_eg.second->back(), dirt_replan_query->goal_state));
        closest_node->indices.push_back(index++);
      }
      std::sort(closest_node->indices.begin(), closest_node->indices.end(),
                [this, pred_values](const int& a, const int& b) { return pred_values[a] > pred_values[b]; });
    }
    prx::time_profiler_t::measure(_resolve_profiler);  // blossom

    // is_blossom_expand = (closest_node->edge_generators.size() > 1);
    is_blossom_expand = (closest_node->is_blossom_expand_done) && !closest_node->random_expand;
    std::pair<plan_t*, trajectory_t*> eg = std::make_pair(nullptr, nullptr);

    double new_node_dir_radius;
    std::vector<dirt_replan_node_t*> dir_updates;

    if (closest_node->indices.size() == 0)
    {
      // This can happen if you are not using default expand (like curate).
      // Essentially, that procedure fails for some reason, and you end up adding no edge to the tree.
      // So we increment the counter for blossom expand collisions.
      _blossom_edges_counter.collision_check += dirt_spec->blossom_number;
    }

    prx::time_profiler_t::measure(_resolve_profiler);  // expand

    while (closest_node->indices.size() != 0)
    {
      eg = closest_node->edge_generators[closest_node->indices.back()];
      closest_node->edge_generators[closest_node->indices.back()] = std::make_pair(nullptr, nullptr);
      const double cost_edge{ cost_function(*eg.second, *eg.first) };
      const double g_closest{ closest_node->cost_to_come + cost_edge };
      const double h_closest{ _heuristic(eg.second->back(), dirt_replan_query->goal_state) };
      const double f_closest{ _f_function(g_closest, h_closest) };
      closest_node->indices.pop_back();

      // bnb (based on g): Do not add a new node if g(node) > g(current_sln)
      if ((goal_vertex != start_vertex && g_closest > _stats.current_solution_cost))
      {
        delete eg.first;
        delete eg.second;
        eg = std::make_pair(nullptr, nullptr);
        if (is_blossom_expand)
        {
          _blossom_edges_counter.g_rejected++;
        }
        else
        {
          _random_edges_counter.g_rejected++;
        }
        continue;
      }

      // pruning condition
      double parent_distance = distance_function(eg.second->back(), closest_node->point);
      new_node_dir_radius = parent_distance;

      auto prox_nodes = metric->radius_and_closest_query(eg.second->back(), std::max(parent_distance, max_radius));
      dir_updates.clear();
      std::transform(prox_nodes.begin(), prox_nodes.end(), std::back_inserter(dir_updates),
                     [](proximity_node_t* prox_node) { return static_cast<dirt_replan_node_t*>(prox_node); });
      std::for_each(dir_updates.begin(), dir_updates.end(), [&, this](dirt_replan_node_t* node) {
        const double f_node{ _f_function(node->cost_to_come, node->cost_to_go) };
        if (f_closest > f_node)
        {
          new_node_dir_radius = std::min(new_node_dir_radius, distance_function(eg.second->back(), node->point));
        }
      });

      // dirt-pruning
      if (dirt_spec->use_pruning)
      {
        bool delete_node = false;
        std::for_each(dir_updates.begin(), dir_updates.end(), [&, this](dirt_replan_node_t* node) {
          const double f_node{ _f_function(node->cost_to_come, node->cost_to_go) };
          if (!delete_node && f_closest > f_node)
          {
            if (new_node_dir_radius + distance_function(node->point, eg.second->back()) < node->dir_radius)
            {
              delete_node = true;
            }
          }
        });
        if (delete_node)
        {
          delete eg.first;
          delete eg.second;
          eg = std::make_pair(nullptr, nullptr);
          if (is_blossom_expand)
          {
            _blossom_edges_counter.pruning++;
          }
          else
          {
            _random_edges_counter.pruning++;
          }
          continue;
        }
      }

      // validity check
      bool valid = valid_check(*eg.second);

      if (!valid)
      {
        delete eg.first;
        delete eg.second;
        eg = std::make_pair(nullptr, nullptr);
        if (is_blossom_expand)
        {
          _blossom_edges_counter.collision_check++;
        }
        else
        {
          _random_edges_counter.collision_check++;
        }
        continue;
      }
      else
      {
        if (is_blossom_expand)
        {
          _blossom_edges_counter.accepted++;
        }
        else
        {
          _random_edges_counter.accepted++;
        }
      }

      break;
    }
    if (eg.first != nullptr)
    {
      add_edge_to_tree(eg, closest_node, dir_updates, new_node_dir_radius);
      delete eg.first;
      delete eg.second;
    }

    _stats.total_iterations++;
  } while (!condition->check());
  if (dirt_spec->verbose)
  {
    print_statistics();
  }
}

node_index_t dirt_replan_t::add_contingency(std::pair<plan_t*, trajectory_t*> eg, dirt_replan_node_t* closest_node,
                                            std::vector<dirt_replan_node_t*> dir_updates, double new_node_dir_radius)
{
  PRX_MSG("Adding contingency")
  const double& node_duration{ closest_node->cost_to_come };  // Assuming G == duration
  // PRX_DBG_VARS(*eg.first);
  // double curr_duration;
  // while(node_duration)
}

node_index_t dirt_replan_t::update_tree(std::pair<plan_t*, trajectory_t*> eg, dirt_replan_node_t* closest_node,
                                        std::vector<dirt_replan_node_t*> dir_updates, double new_node_dir_radius)
{
  auto node_index = tree().add_vertex<dirt_replan_node_t, rrt_edge_t>();
  auto new_tree_node = tree().get_vertex_as<dirt_replan_node_t>(node_index);
  new_tree_node->point = state_space->clone_point(eg.second->back());
  new_tree_node->bridge = true;
  edge_index_t edge_index = tree().add_edge(closest_node->get_index(), node_index);
  auto new_edge = tree().get_edge_as<rrt_edge_t>(edge_index);
  new_edge->plan = std::make_shared<plan_t>(*eg.first);
  new_edge->traj = std::make_shared<trajectory_t>(*eg.second);
  new_edge->edge_cost = cost_function(*eg.second, *eg.first);

  new_tree_node->cost_to_come = closest_node->cost_to_come + new_edge->edge_cost;
  new_tree_node->cost_to_go = _heuristic(eg.second->back(), dirt_replan_query->goal_state);
  const double f_new_node{ _f_function(new_tree_node->cost_to_come, new_tree_node->cost_to_go) };

  new_tree_node->blossom_number = dirt_spec->blossom_number;
  new_tree_node->dir_radius = new_node_dir_radius;

  max_radius = std::max(max_radius, new_node_dir_radius);
  // EXPERIMENTAL: Try commenting this line out. Behavior seems reasonable, but need to consider theoretical effects
  // get_vertex(start_vertex)->dir_radius = max_radius;

  std::for_each(dir_updates.begin(), dir_updates.end(), [&, this](dirt_replan_node_t* node) {
    const double sibling_distance = distance_function(node->point, new_tree_node->point);
    const double f_node{ _f_function(node->cost_to_come, node->cost_to_go) };
    if (f_new_node < f_node)
    {
      node->dir_radius = std::min(node->dir_radius, sibling_distance);
      if (dirt_spec->use_pruning && node->dir_radius + sibling_distance < new_node_dir_radius && !node->is_safe)
      {
        if (!node->bridge)
        {
          metric->remove_node(node);
          node->bridge = true;
        }
        node_index_t iter = node->get_index();
        while (is_leaf(iter) && get_vertex(iter)->bridge && !is_best_goal(iter))
        {
          node_index_t next = get_vertex(iter)->get_parent();
          for (int man_index = 0; man_index < get_vertex(iter)->indices.size(); man_index++)
          {
            delete get_vertex(iter)->edge_generators[get_vertex(iter)->indices[man_index]].first;
            delete get_vertex(iter)->edge_generators[get_vertex(iter)->indices[man_index]].second;
          }
          get_vertex(iter)->edge_generators.clear();
          get_vertex(iter)->indices.clear();
          get_vertex(iter)->is_blossom_expand_done = false;
          get_vertex(iter)->random_expand = false;
          remove_leaf(iter);
          iter = next;
        }
      }
    }
  });
  if (new_tree_node->cost_to_go < closest_node->cost_to_go)
  {
    child_extension = true;
    previous_child = node_index;
  }
  metric->add_node(new_tree_node.get());
  new_tree_node->bridge = false;
  update_goal(node_index);
  return node_index;
}

node_index_t dirt_replan_t::add_edge_to_tree(std::pair<plan_t*, trajectory_t*> eg, dirt_replan_node_t* closest_node,
                                             std::vector<dirt_replan_node_t*> dir_updates, double new_node_dir_radius)

{
  node_index_t node_index;
  const double& node_duration{ closest_node->cost_to_come };  // Assuming G == duration
  if (dirt_spec->use_contingency and node_duration < dirt_spec->contingency_radius)
  {
    node_index = add_contingency(eg, closest_node, dir_updates, new_node_dir_radius);
  }
  else
  {
    node_index = update_tree(eg, closest_node, dir_updates, new_node_dir_radius);
  }
  return node_index;
}

void dirt_replan_t::update_goal(node_index_t node_index)
{
  auto new_tree_node = tree().get_vertex_as<dirt_replan_node_t>(node_index);
  const double g_value{ new_tree_node->cost_to_come };
  const double h_value{ new_tree_node->cost_to_go };
  const double f_new_node{ _f_function(g_value, h_value) };

  if (_current_solution_type == dirt_replan_query_t::solution_type_t::MIN_F_VALUE)
  {
    if (not _stats.solution_found and _best_f_value > f_new_node)
    {
      _best_f_node = node_index;
      _best_f_value = f_new_node;
    }
    if ((not _stats.solution_found or g_value < _stats.current_solution_cost) and
        dirt_replan_query->goal_check(new_tree_node->point))
    {
      _best_f_node = node_index;
      _best_f_value = f_new_node;
      _stats.update_solution(g_value, _timer.measure());

      if (dirt_spec->verbose)
      {
        std::cout << "[dirt] Found new goal: " << state_space->print_point(new_tree_node->point, 3);
        std::cout << " cost:" << _stats.current_solution_cost;
        std::cout << " time:" << _stats.current_solution_time;
        std::cout << " iter:" << _stats.current_solution_iterations;
        std::cout << " nodes:" << metric->get_nr_nodes() << "\n";
      }
      bnb(start_vertex, _stats.current_solution_cost);
    }
  }
  else if (_current_solution_type == dirt_replan_query_t::solution_type_t::TREE_TRAJECTORY)
  {
    if (g_value < _best_f_value and dirt_replan_query->goal_check(new_tree_node->point))
    {
      _best_f_node = node_index;
      _best_f_value = g_value;
      _stats.update_solution(g_value, _timer.measure());
      if (dirt_spec->verbose)
      {
        std::cout << "[dirt] Found new goal: " << state_space->print_point(new_tree_node->point, 3);
        std::cout << " cost:" << _stats.current_solution_cost;
        std::cout << " time:" << _stats.current_solution_time;
        std::cout << " iter:" << _stats.current_solution_iterations;
        std::cout << " nodes:" << metric->get_nr_nodes() << "\n";
      }
      bnb(start_vertex, _stats.current_solution_cost);
    }
  }
}

std::vector<double> dirt_replan_t::get_statistics()
{
  // time, iters, nodes, solution quality, first_time, first_iters, current_solution,
  std::vector<double> rrt_statistics = rrt_t::get_statistics();
  std::vector<double> rand_counts(
      { static_cast<double>(_random_edges_counter.bnb), static_cast<double>(_random_edges_counter.g_rejected),
        static_cast<double>(_random_edges_counter.pruning), static_cast<double>(_random_edges_counter.collision_check),
        static_cast<double>(_random_edges_counter.accepted) });

  std::vector<double> blossom_counts({ static_cast<double>(_blossom_edges_counter.bnb),
                                       static_cast<double>(_blossom_edges_counter.g_rejected),
                                       static_cast<double>(_blossom_edges_counter.pruning),
                                       static_cast<double>(_blossom_edges_counter.collision_check),
                                       static_cast<double>(_blossom_edges_counter.accepted) });
  rrt_statistics.insert(rrt_statistics.end(), std::make_move_iterator(rand_counts.begin()),
                        std::make_move_iterator(rand_counts.end()));
  rrt_statistics.insert(rrt_statistics.end(), std::make_move_iterator(blossom_counts.begin()),
                        std::make_move_iterator(blossom_counts.end()));
  return rrt_statistics;
}

void dirt_replan_t::_reset()
{
  // clear the stuff
  tree().purge();
  if (metric != nullptr)
  {
    delete metric;
    metric = nullptr;
  }
}

bool dirt_replan_t::tree_solution(const node_index_t goal_node_idx)
{
  if (goal_node_idx == start_vertex)
    return false;

  rrt_query->solution_cost = tree().get_vertex_as<rrt_node_t>(goal_node_idx)->cost_to_come;
  std::deque<node_index_t> node_indices;
  node_index_t current_index = goal_node_idx;
  while (current_index != start_vertex)
  {
    auto node = get_vertex(current_index);
    node_indices.push_front(current_index);
    current_index = tree()[current_index]->get_parent();
  }
  rrt_query->solution_plan.clear();
  rrt_query->solution_traj.clear();

  // Adding the start state
  rrt_query->solution_traj.push_back(tree().get_vertex_as<rrt_node_t>(start_vertex)->point);

  // for (int i = 0; i < node_indices.size(); i++)
  for (auto& node_idx : node_indices)
  {
    const node_index_t parent_edge_idx{ tree()[node_idx]->get_parent_edge() };
    const std::shared_ptr<rrt_edge_t> parent_edge{ tree().get_edge_as<rrt_edge_t>(parent_edge_idx) };
    rrt_query->solution_plan += *(parent_edge->plan);

    // Start with i=1 because parent->traj[0] is the same state as grandparent->traj[-1]
    for (int i = 1; i < parent_edge->traj->size(); ++i)
    {
      rrt_query->solution_traj.push_back(parent_edge->traj->at(i));
    }
  }
  return true;
}

void dirt_replan_t::_fulfill_query()
{
  prx::time_profiler_t::reset(_fulfill_profiler);

  bool solution_found{ tree_solution(_best_f_node) };

  prx::time_profiler_t::measure(_fulfill_profiler);  // best node

  if (not solution_found)
  {
    if (dirt_spec->verbose)
      std::cout << "No solution found during planning cycle. # of nodes: " << metric->get_nr_nodes() << "\n";
    rrt_query->solution_cost = 0;
  }

  if (rrt_query->get_visualization)
  {
    if (dirt_spec->verbose)
      std::cout << "Visualizing " << tree().num_edges() << " edges" << std::endl;
    auto iter_bounds = tree().edges();
    for (auto iter = iter_bounds.first; iter != iter_bounds.second; iter++)
    {
      const auto curr_traj = tree().get_edge_as<rrt_edge_t>((*iter)->get_index())->traj;
      rrt_query->tree_visualization.push_back(*curr_traj);
    }
  }
}

void dirt_replan_t::bnb(node_index_t v, double cost_bound, bool delete_flag)
{
  if (not dirt_spec->bnb)
    return;

  auto node = get_vertex(v);
  const double f_value{ _f_function(node->cost_to_come, node->cost_to_go) };
  bool res = delete_flag || f_value > cost_bound;
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
    for (int man_index = 0; man_index < node->indices.size(); man_index++)
    {
      if (node->random_expand)
      {
        _random_edges_counter.bnb++;
      }
      else
      {
        _blossom_edges_counter.bnb++;
      }
      delete node->edge_generators[node->indices[man_index]].first;
      delete node->edge_generators[node->indices[man_index]].second;
    }
    node->edge_generators.clear();
    node->indices.clear();

    // remove the node
    tree().remove_vertex(v);
  }
}
}  // namespace prx
