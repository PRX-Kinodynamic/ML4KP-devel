#include "prx/planning/planners/aorrt.hpp"

namespace prx
{
aorrt_t::aorrt_t(const std::string& new_name) : rrt_t(new_name), _c_max(0.0)
{
  metric = nullptr;
  planner_name = new_name;

  _cost_memory.push_back(new double(0.0));
  _cost_state_space = new space_t("E", _cost_memory, "cost_state");  // { &_theta1, &_theta1dot };
  _cost_state_space->set_bounds({ 0.0 }, { 0.0 });
  _cost_aux1 = _cost_state_space->make_point();
  _cost_aux2 = _cost_state_space->make_point();
}
aorrt_t::~aorrt_t()
{
  _reset();
}
void aorrt_t::_link_and_setup_spec(planner_specification_t* spec)
{
  // reset is always called before this
  aorrt_spec = dynamic_cast<aorrt_specification_t*>(spec);
  prx_assert(aorrt_spec != nullptr, "RRT_2 received an incorrect specification.");
  distance_function = aorrt_spec->distance_function;
  cost_function = aorrt_spec->cost_function;
  sample_state = aorrt_spec->sample_state;
  sample_plan = aorrt_spec->sample_plan;
  valid_check = aorrt_spec->valid_check;
  valid_stop_check = aorrt_spec->valid_stop_check;
  propagate = aorrt_spec->propagate;
  use_replanning = aorrt_spec->use_replanning;
  _bnb = aorrt_spec->bnb;

  X_state_space = aorrt_spec->state_space;
  control_space = aorrt_spec->control_space;
  expand = aorrt_spec->expand;

  Y_state_space = new space_t({ X_state_space, _cost_state_space });
  state_space = Y_state_space;

  metric = new graph_nearest_neighbors_t(Y_distance_function);

  // Sample pts
  X_sample_point = X_state_space->make_point();
  Y_sample_point = Y_state_space->make_point();
  _cost_sample_point = _cost_state_space->make_point();

  // Auxiliary pts
  X_aux_pt = X_state_space->make_point();
  Y_aux_pt = Y_state_space->make_point();
  _X_aux1 = X_state_space->make_point();
  _X_aux2 = X_state_space->make_point();
  _cost_aux_pt = _cost_state_space->make_point();

  // Min pt
  Y_min = Y_state_space->make_point();

  _stats.current_solution_cost = aorrt_spec->c_max;
}
bool aorrt_t::_preprocess()
{
  _tree.allocate_memory<aorrt_node_t, aorrt_edge_t>(1000);
  return true;
}
bool aorrt_t::_link_and_setup_query(planner_query_t* query)
{
  aorrt_query = dynamic_cast<aorrt_query_t*>(query);
  prx_assert(aorrt_query != nullptr, "AORRT received an incorrect query type.");

  if (_tree.num_vertices() == 0 ||
      !X_state_space->equal_points(_tree.get_vertex_as<aorrt_node_t>(start_vertex)->point, aorrt_query->start_state))
  {
    // clear existing data structure
    metric->clear();
    _tree.clear();
    start_vertex = _tree.add_vertex<aorrt_node_t, aorrt_edge_t>();
    goal_vertex = start_vertex;
    auto start_node = _tree.get_vertex_as<aorrt_node_t>(start_vertex);
    start_node->point = Y_state_space->make_point();

    _cost_state_space->copy(_cost_aux_pt, { 0.0 });
    Y_state_space->point_union(aorrt_query->start_state, _cost_aux_pt, start_node->point);

    start_node->cost_to_come = 0;
    metric->add_node(start_node.get());
  }
  _timer.reset();
  _stats.reset();
  // iteration_count = 0;
  // current_solution_iters = 0;
  // current_solution_time = 0;

  return true;
}

void aorrt_t::_resolve_query(condition_check_t* condition)
{
  bool sol_found = false;
  _w_x_bk = aorrt_spec->w_x;
  _w_c_bk = aorrt_spec->w_c;

  // At first, explore only the state space. When a solution is found, switch to State \times Cost space
  aorrt_spec->w_x = 1.0;
  aorrt_spec->w_c = 0.0;

  do
  {
    // sample state
    sample_state(X_sample_point);

    _cost_state_space->sample(_cost_sample_point);
    Y_state_space->point_union(X_sample_point, _cost_sample_point, Y_sample_point);

    // find closest
    auto closest_node = static_cast<aorrt_node_t*>(metric->single_query(Y_sample_point));
    Y_state_space->split_point(closest_node->point, X_aux_pt, _cost_aux_pt);

    // sample_plan(plan);
    std::vector<plan_t*> plans;
    std::vector<trajectory_t*> trajs;
    expand(X_aux_pt, plans, trajs, aorrt_spec->blossom_number, false);
    plan_t plan(*plans.front());
    trajectory_t traj(*trajs.front());

    _c_new = _cost_aux_pt->at(0) + cost_function(traj, plan);

    if (_c_new < _stats.current_solution_cost and valid_check(traj))
    {
      // The new cost is higher than the max in the tree but lower than min found to the goal
      // This happens when no solution has been found or if it is allowed to explore higher costs than the current goal
      // if (_c_new > _c_max)
      if (goal_vertex == start_vertex and _c_new > _c_max)
      {
        _c_max = _c_new;
        _cost_state_space->set_bounds({ 0.0 }, { _c_max });
      }
      // add vertex
      auto node_index = _tree.add_vertex<aorrt_node_t, aorrt_edge_t>();
      auto new_tree_node = _tree.get_vertex_as<aorrt_node_t>(node_index);
      _cost_state_space->copy(_cost_aux_pt, { _c_new });
      Y_state_space->point_union(traj.back(), _cost_aux_pt, Y_aux_pt);
      new_tree_node->point = Y_state_space->clone_point(Y_aux_pt);
      new_tree_node->cost_to_come = _c_new;

      // add node to metric
      metric->add_node(new_tree_node.get());

      // add edge
      edge_index_t edge_index = _tree.add_edge(closest_node->get_index(), node_index);
      auto new_edge = _tree.get_edge_as<aorrt_edge_t>(edge_index);
      new_edge->plan = std::make_shared<plan_t>(plan);
      new_edge->traj = std::make_shared<trajectory_t>(traj);

      update_goal(node_index);
    }
    _stats.total_iterations++;
  } while (!condition->check());
}

void aorrt_t::_fulfill_query()
{
  if (goal_vertex != start_vertex && !use_replanning)
  {
    // backtrack to get the plan and trajectory
    aorrt_query->solution_cost = _stats.current_solution_cost;
    std::deque<node_index_t> node_indices;
    node_index_t current_index = goal_vertex;

    trajectory_costs.clear();
    aorrt_query->solution_traj.clear();
    aorrt_query->solution_plan.clear();

    while (current_index != start_vertex)
    {
      node_indices.push_front(current_index);
      current_index = _tree[current_index]->get_parent();
    }

    aorrt_query->solution_plan = *_tree.get_edge_as<aorrt_edge_t>(_tree[node_indices[0]]->get_parent_edge())->plan;
    aorrt_query->solution_traj = *_tree.get_edge_as<aorrt_edge_t>(_tree[node_indices[0]]->get_parent_edge())->traj;

    trajectory_costs.push_back(_tree.get_vertex_as<aorrt_node_t>(node_indices[0])->point);

    for (int i = 1; i < node_indices.size(); i++)
    {
      aorrt_query->solution_traj.resize(aorrt_query->solution_traj.size() - 1);
      aorrt_query->solution_plan += *_tree.get_edge_as<aorrt_edge_t>(_tree[node_indices[i]]->get_parent_edge())->plan;
      aorrt_query->solution_traj += *_tree.get_edge_as<aorrt_edge_t>(_tree[node_indices[i]]->get_parent_edge())->traj;

      trajectory_costs.push_back(_tree.get_vertex_as<aorrt_node_t>(node_indices[i])->point);
    }
  }
  else
  {
    aorrt_query->solution_cost = 0;
  }
  if (use_replanning)
  {
    std::cout << "REPLANNING NOT IMPLEMENTED" << std::endl;
  }
  if (aorrt_query->get_visualization)
  {
    auto iter_bounds = _tree.edges();
    for (auto iter = iter_bounds.first; iter != iter_bounds.second; iter++)
    {
      aorrt_query->tree_visualization.push_back(*_tree.get_edge_as<aorrt_edge_t>((*iter)->get_index())->traj);
    }
  }
}

std::vector<double> aorrt_t::get_statistics()
{
  // time, iters, nodes, solution quality, first_time, first_iters, current_solution
  return { _timer.measure(),
           static_cast<double>(_stats.total_iterations),
           static_cast<double>(metric->get_nr_nodes()),
           _stats.current_solution_cost,
           _stats.current_solution_time,
           static_cast<double>(_stats.current_solution_iterations) };
}

void aorrt_t::_reset()
{
  // clear the stuff
  _tree.purge();
  if (metric != nullptr)
  {
    delete metric;
    metric = nullptr;
  }
}

void aorrt_t::update_goal(node_index_t node_index)
{
  auto new_tree_node = _tree.get_vertex_as<aorrt_node_t>(node_index);
  auto tree_edge = _tree.get_edge_as<aorrt_edge_t>(new_tree_node->get_parent_edge());

  space_point_t pt = tree_edge->traj->back();
  // if(distance_function(aorrt_query->goal_state, traj.back()) < aorrt_query->goal_region_radius
  if (aorrt_query->goal_check(pt) && _c_new < _stats.current_solution_cost)
  {
    // statics
    // current_solution = _c_new;
    // current_solution_time = _timer.measure();
    // current_solution_iters = iteration_count;

    // Update the solution
    Y_state_space->copy(Y_min, Y_aux_pt);
    _stats.current_solution_cost = _c_new;
    goal_vertex = node_index;

    _stats.update_solution(_c_new, _timer.measure());

    aorrt_spec->c_max = _c_new;
    aorrt_spec->w_x = _w_x_bk;
    aorrt_spec->w_c = _w_c_bk;

    std::cout << "[" + planner_name + "] Found new goal:( " << pt << ")";
    std::cout << " cost:" << _stats.current_solution_cost;
    std::cout << " time:" << _stats.current_solution_time;
    std::cout << " iter:" << _stats.current_solution_iterations;
    std::cout << " nodes:" << metric->get_nr_nodes();
    std::cout << "\n";

    _cost_state_space->set_bounds({ 0.0 }, { aorrt_spec->c_max });
    if (_bnb)
    {
      bnb(start_vertex, _c_new * aorrt_spec->cost_multiplier);
    }
  }
}

void aorrt_t::bnb(node_index_t v, double cost_bound, bool delete_flag)
{
  auto node = _tree.get_vertex_as<aorrt_node_t>(v);
  Y_state_space->split_point(node->point, X_aux_pt, _cost_aux_pt);
  const double node_cost = _cost_aux_pt->at(0);
  const bool res = delete_flag || (node_cost > cost_bound);
  std::list<node_index_t> children = node->get_children();

  for (auto child : children)
  {
    bnb(child, cost_bound, res);
  }
  children = node->get_children();

  if (res && children.empty())
  {
    // remove the node
    metric->remove_node(node.get());
    if (_bnb)
    {
      _tree.remove_vertex(v);
    }
  }
}

}  // namespace prx
