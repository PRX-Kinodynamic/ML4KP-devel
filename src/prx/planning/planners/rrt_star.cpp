#include "prx/planning/planners/rrt_star.hpp"
#include "prx/utilities/general/timed_logger.hpp"

namespace prx
{
rrt_star_t::rrt_star_t(const std::string& new_name) : planner_t(new_name)
{
  _metric = nullptr;
  _planner_name = new_name;
}
rrt_star_t::~rrt_star_t()
{
  _reset();
}
void rrt_star_t::_link_and_setup_spec(planner_specification_t* spec)
{
  // reset is always called before this
  _rrt_star_spec = dynamic_cast<rrt_star_specification_t*>(spec);
  prx_assert(_rrt_star_spec != nullptr, "RRT received an incorrect specification.");
  _distance_function = _rrt_star_spec->distance_function;
  _cost_function = _rrt_star_spec->cost_function;
  _valid_check = _rrt_star_spec->valid_check;
  _steer_function = _rrt_star_spec->steer_function;

  _state_space = _rrt_star_spec->state_space;
  _x_new = _state_space->make_point();
  _x_rand = _state_space->make_point();
  _control_space = _rrt_star_spec->control_space;

  _metric = new graph_nearest_neighbors_t(_distance_function);
  // we now have spaces and necessary functions
  prx_assert(_rrt_star_spec->eta_min > 0.0, "[" << _planner_name << "]: `eta_min` must be greater than 0 ");
  prx_assert(_rrt_star_spec->eta_max > _rrt_star_spec->eta_min,
             "[" << _planner_name << "]: `eta_max` must be greater than `eta_min` ");
}

bool rrt_star_t::_preprocess()
{
  _tree.allocate_memory<rrt_star_node_t, rrt_star_edge_t>(1000);
  return true;
}

bool rrt_star_t::_link_and_setup_query(planner_query_t* query)
{
  _rrt_star_query = dynamic_cast<rrt_star_query_t*>(query);
  prx_assert(_rrt_star_query != nullptr, "RRT received an incorrect query type.");
  if (_tree.num_vertices() == 0 ||
      !_state_space->equal_points(_tree.get_vertex_as<rrt_star_node_t>(_start_vertex)->point,
                                  _rrt_star_query->start_state))
  {
    // clear existing data structure
    _metric->clear();
    _tree.clear();
    _start_vertex = _tree.add_vertex<rrt_star_node_t, rrt_star_edge_t>();
    _goal_vertex = _start_vertex;
    auto start_node = _tree.get_vertex_as<rrt_star_node_t>(_start_vertex);
    start_node->point = _state_space->clone_point(_rrt_star_query->start_state);
    start_node->cost_to_come = 0;
    _metric->add_node(start_node.get());
  }
  _goal_region_radius = _rrt_star_query->goal_region_radius;
  _goal_state = _rrt_star_query->goal_state;
  _timer.reset();
  _iteration_count = 0;
  _current_solution = 0;
  _current_solution_iters = 0;
  _current_solution_time = 0;
  const std::size_t dim{ _state_space->get_dimension() };
  k_RRT = static_cast<int>(std::pow(2, dim) * std::exp(1.0 + 1.0 / dim) + 1);

  return true;
}

void rrt_star_t::_resolve_query(condition_check_t* condition)
{
  std::shared_ptr<rrt_star_node_t> x_new{ nullptr };

  std::size_t nodes_added{ _tree.num_vertices() };
using namespace std::literals::chrono_literals;
std::ofstream ss(prx::out_path + "rrt_star.log", std::ofstream::trunc);
   prx::utilities::timed_logger_t timed_logger(1000ms, ss);
  
   timed_logger.add("iterations", &_iteration_count);
   timed_logger.add("nodes", &nodes_added);
   timed_logger.run();
   do
  {
    // samples
    _rrt_star_spec->sample_state(_x_rand);
    _eta = uniform_random(_rrt_star_spec->eta_min, _rrt_star_spec->eta_max);

    // Find nearest
    rrt_star_node_t* x_nearest{ static_cast<rrt_star_node_t*>(_metric->single_query(_x_rand)) };

    // traj <- Steer(x_nearest, x_rand)
    trajectory_t traj(_state_space);
    _steer_function(traj, x_nearest->point, _x_rand, _eta);

    // collision check
    if (_valid_check(traj))
    {
      nodes_added++;
      // V <- V \cup \{ x_new \}
      add_to_tree(x_new, traj.back());
      // X_near <- kNearest(G=(V,E), x_new, k_RRT & log(i))
      const CloseNodes X_near{ get_near_nodes(x_new, nodes_added) };

      // x_min <- x_nearest
      rrt_star_node_t* x_min{ x_nearest };

      // Connect along the minimum-cost path
      const double edge_cost{ connect_along_minimum_cost(x_min, x_new, X_near, traj) };

      // E <- E \cup { x_min, x_new }
      add_edge(x_new, x_min, traj, edge_cost);

      // Rewire the tree
      rewire_tree(X_near, x_new);

      update_goal(x_new->get_index());
    }
    _iteration_count++;
  } while (!condition->check());
   timed_logger.stop();
}

void rrt_star_t::add_to_tree(NodePtr& x_new, space_point_t state)
{
  const node_index_t x_new_idx{ _tree.add_vertex<rrt_star_node_t, rrt_star_edge_t>() };
  x_new = _tree.get_vertex_as<rrt_star_node_t>(x_new_idx);
  x_new->point = _state_space->clone_point(state);
}

rrt_star_t::CloseNodes rrt_star_t::get_near_nodes(NodePtr& x_new, const std::size_t nodes_added)
{
  const int k_nearest{ static_cast<int>(k_RRT * std::log10(nodes_added)) };
  const CloseNodes X_near{ _metric->multi_query(x_new->point, k_nearest) };
  return X_near;
}

double rrt_star_t::connect_along_minimum_cost(rrt_star_node_t*& x_min, NodePtr& x_new, const CloseNodes& X_near,
                                              trajectory_t& traj)
{
  double edge_cost{ _distance_function(x_min->point, x_new->point) };
  double c_min{ x_min->cost_to_come + edge_cost };

  // Connect along the minimum-cost path
  for (auto x_near_abstract : X_near)
  {
    rrt_star_node_t* x_near{ static_cast<rrt_star_node_t*>(x_near_abstract) };
    const double cost_edge_near_new{ _distance_function(x_near->point, x_new->point) };
    const double cost_x_near{ x_near->cost_to_come };
    const double cost_x_new_via_x_near{ cost_x_near + cost_edge_near_new };

    trajectory_t traj_near_to_new(_state_space);

    if (cost_x_new_via_x_near < c_min )
    {
	_steer_function(traj_near_to_new, x_near->point, x_new->point, cost_x_new_via_x_near);
	if ( _valid_check(traj_near_to_new))
    {
      x_min = x_near;
      edge_cost = cost_edge_near_new;
      c_min = cost_x_new_via_x_near;
      traj = traj_near_to_new;
    }
    }
  }
  x_new->cost_to_come = c_min;
  _metric->add_node(x_new.get());
  return edge_cost;
}

void rrt_star_t::add_edge(const NodePtr x_new, rrt_star_node_t* x_min, trajectory_t& traj, const double edge_cost)
{
  // E <- E \cup { x_min, x_new }
  const node_index_t x_new_idx{ x_new->get_index() };
  const edge_index_t x_new_edge_idx{ _tree.add_edge(x_min->get_index(), x_new_idx) };
  std::shared_ptr<rrt_star_edge_t> x_new_edge{ _tree.get_edge_as<rrt_star_edge_t>(x_new_edge_idx) };
  x_new_edge->traj = std::make_shared<trajectory_t>(traj);
  x_new_edge->edge_cost = edge_cost;
}

void rrt_star_t::rewire_tree(const CloseNodes& X_near, NodePtr& x_new)
{
  const double cost_x_new{ x_new->cost_to_come };
  const node_index_t x_new_idx{ x_new->get_index() };
  for (auto x_near_abstract : X_near)
  {
    rrt_star_node_t* x_near{ static_cast<rrt_star_node_t*>(x_near_abstract) };
    const double cost_x_near{ x_near->cost_to_come };
    const double cost_edge_near_new{ _distance_function(x_near->point, x_new->point) };
    const double cost_x_near_via_x_new{ cost_x_new + cost_edge_near_new };

    trajectory_t traj_new_to_near(_state_space);

    if (cost_x_near_via_x_new < cost_x_near )
    {
    	_steer_function(traj_new_to_near, x_new->point, x_near->point, cost_x_near_via_x_new);
	if (_valid_check(traj_new_to_near))
    {
      const node_index_t x_near_idx{ x_near->get_index() };
      _tree.transplant(x_near_idx, x_new_idx);
      const edge_index_t x_near_edge_idx{ x_near->get_parent_edge() };
      std::shared_ptr<rrt_star_edge_t> parent_edge{ _tree.get_edge_as<rrt_star_edge_t>(x_near_edge_idx) };
      // traj_new_to_near.pop_back();
      parent_edge->traj = std::make_shared<trajectory_t>(traj_new_to_near);
      parent_edge->edge_cost = cost_edge_near_new;
      x_near->cost_to_come = cost_x_near_via_x_new;
    	}
    }
  }
}

void rrt_star_t::_fulfill_query()
{
  if (_goal_vertex != _start_vertex)
  {
    // backtrack to get the plan and trajectory
    _rrt_star_query->solution_cost = _tree.get_vertex_as<rrt_star_node_t>(_goal_vertex)->cost_to_come;
    std::deque<node_index_t> node_indices;
    node_index_t current_index = _goal_vertex;
    while (current_index != _start_vertex)
    {
      node_indices.push_front(current_index);
      current_index = _tree[current_index]->get_parent();
    }
    // node_indices.push_front(current_index);
    _rrt_star_query->solution_traj.clear();
    // const edge_index_t parent_edge{ _tree[node_indices[0]]->get_parent_edge() };
    // _rrt_star_query->solution_traj = *_tree.get_edge_as<rrt_star_edge_t>(parent_edge)->traj;
    for (auto sln_idx : node_indices)
    {
      const edge_index_t parent_edge{ _tree[sln_idx]->get_parent_edge() };
      const std::shared_ptr<rrt_star_edge_t> edge{ _tree.get_edge_as<rrt_star_edge_t>(parent_edge) };
      _rrt_star_query->solution_traj += *(edge->traj);
    }
  }
  else
  {
    _rrt_star_query->solution_cost = 0;
  }
  if (_rrt_star_query->get_visualization)
  {
    auto iter_bounds = _tree.edges();
    for (auto iter = iter_bounds.first; iter != iter_bounds.second; iter++)
    {

      	    const edge_index_t edge_idx{ (*iter)->get_index() };
	    const std::shared_ptr<rrt_star_edge_t> edge{ _tree.get_edge_as<rrt_star_edge_t>(edge_idx) };
      const std::shared_ptr<trajectory_t> traj{ edge->traj };
      _rrt_star_query->tree_visualization.push_back(*traj);
    }
  }
}

std::vector<std::string> rrt_star_t::get_statistics_header()
{
  return { "time", "iters", "nodes", "solution_cost", "solution_time", "solution_iters" };
}
std::vector<double> rrt_star_t::get_statistics()
{
  // time, iters, nodes, solution quality, first_time, first_iters, current_solution
  return { _timer.measure(),
           static_cast<double>(_iteration_count),
           static_cast<double>(_metric->get_nr_nodes()),
           _current_solution,
           _current_solution_time,
           static_cast<double>(_current_solution_iters) };
}

void rrt_star_t::_reset()
{
  // clear the stuff
  _tree.purge();
  if (_metric != nullptr)
  {
    delete _metric;
    _metric = nullptr;
  }
}

void rrt_star_t::update_goal(const node_index_t goal_index)
{
  std::shared_ptr<rrt_star_node_t> proposed_new_goal_node{ _tree.get_vertex_as<rrt_star_node_t>(goal_index) };
  std::shared_ptr<rrt_star_node_t> current_goal_node{ _tree.get_vertex_as<rrt_star_node_t>(_goal_vertex) };
  if (_rrt_star_query->goal_check(proposed_new_goal_node->point))
  {
    if (_goal_vertex == _start_vertex || current_goal_node->cost_to_come > proposed_new_goal_node->cost_to_come)
    {
      _current_solution = proposed_new_goal_node->cost_to_come;
      _current_solution_time = _timer.measure();
      _current_solution_iters = _iteration_count;
      _goal_vertex = goal_index;
      std::cout << "[" + _planner_name + "] Found new goal:" << proposed_new_goal_node->point;
      std::cout << " cost:" << proposed_new_goal_node->cost_to_come;
      std::cout << " time:" << _current_solution_time;
      std::cout << " iter:" << _current_solution_iters;
      std::cout << " nodes:" << _metric->get_nr_nodes() << "\n";
    }
  }
}

void rrt_star_t::print_statistics()
{
  std::cout << "[" + planner_name + "]";
  std::cout << " time:" << _timer.measure();
  std::cout << " iter:" << _iteration_count;
  std::cout << " nodes:" << _metric->get_nr_nodes() << std::endl;
}

void rrt_star_t::from_files(const std::string file_prefix, const std::string directory)
{
  using prx::constants::separating_value;
  using prx::utilities::csv_reader_t;

  using Line = std::vector<double>;
  using Block = std::vector<Line>;
  using Columns = std::vector<std::size_t>;
  using ColumnsQuery = std::vector<Columns>;

  const std::string filename_trajs{ directory + "/" + file_prefix + "_trajectories.txt" };
  const std::string filename_tree{ directory + "/" + file_prefix + "_tree.txt" };

  PRX_DEBUG_VAR_1(filename_trajs);
  PRX_DEBUG_VAR_1(filename_tree);
  _tree.from_file<rrt_star_node_t, rrt_star_edge_t>(filename_tree, _state_space);

  std::unordered_map<std::size_t, std::shared_ptr<prx::trajectory_t>> map_edges_traj;
  csv_reader_t reader_trajs(filename_trajs);

  const Columns c0{ 0 };
  Columns c1{};
  for (int i = 0; i < _state_space->size(); ++i)
  {
    c1.emplace_back(i + 1);
  }
  const ColumnsQuery column_query{ c0, c1 };
  reader_trajs.next_line();  // Remove first line (header)
  while (reader_trajs.has_next_line())
  {
    Block block_traj{ reader_trajs.next_block<double>() };
    if (block_traj.size() == 0)
    {
      continue;
    }
    const std::size_t edge_id{ static_cast<size_t>(block_traj[0][0]) };
    split_block(block_traj, column_query);
    map_edges_traj.insert({ edge_id, std::make_shared<trajectory_t>(_state_space, block_traj) });
  }

  auto vertices_iters = _tree.vertices();
  for (auto iter = vertices_iters.first; iter != vertices_iters.second; ++iter)
  {
    const node_index_t node_idx{ (*iter)->get_index() };
    const node_index_t parent_idx{ (*iter)->get_parent() };
    if (parent_idx == node_idx)
    {
      _start_vertex = node_idx;
      const std::shared_ptr<rrt_star_node_t> node{ _tree.get_vertex_as<rrt_star_node_t>(_start_vertex) };
      node->cost_to_come = 0;
      _state_space->copy(_rrt_star_query->start_state, node->point);

      break;
    }
  }

  std::queue<node_index_t> queue{ { _start_vertex } };

  while (not queue.empty())
  {
    const node_index_t node_idx{ queue.front() };
    const std::shared_ptr<rrt_star_node_t> node{ _tree.get_vertex_as<rrt_star_node_t>(node_idx) };
    if (_start_vertex != node_idx)
    {
      _metric->add_node(node.get());
    }

    // queue children
    for (auto child_idx : node->get_children())
    {
      const std::shared_ptr<rrt_star_node_t> child_node{ _tree.get_vertex_as<rrt_star_node_t>(child_idx) };
      const edge_index_t edge_idx{ child_node->get_parent_edge() };
      const std::shared_ptr<rrt_star_edge_t> edge{ _tree.get_edge_as<rrt_star_edge_t>(edge_idx) };

      const double edge_cost{ _distance_function(node->point, child_node->point) };

      edge->traj = map_edges_traj[edge_idx];
      edge->edge_cost = edge_cost;
      child_node->cost_to_come = node->cost_to_come + edge_cost;

      queue.emplace(child_idx);
    }
    queue.pop();
  }
}

void rrt_star_t::to_files(const std::string file_prefix, const std::string directory)
{
  using prx::constants::separating_value;

  const std::string filename_trajs{ directory + "/" + file_prefix + "_trajectories.txt" };
  const std::string filename_tree{ directory + "/" + file_prefix + "_tree.txt" };

  std::cout << "Saving trajectories as: \n\t" << filename_trajs << "\n";
  std::ofstream ofs_trajs{ filename_trajs.c_str(), std::ofstream::trunc };
  _tree.to_file(filename_tree);

  std::queue<node_index_t> queue{ { _start_vertex } };

  ofs_trajs << "#" << separating_value;
  ofs_trajs << "edge_idx" << separating_value;
  ofs_trajs << "state\n";

  auto pair_edges = _tree.edges();
  for (auto iter = pair_edges.first; iter != pair_edges.second; ++iter)
  {
    const edge_index_t edge_idx{ (*iter)->get_index() };
    const std::shared_ptr<rrt_star_edge_t> edge{ _tree.get_edge_as<rrt_star_edge_t>(edge_idx) };
    const node_index_t child_idx{ edge->get_target() };
    const node_index_t parent_idx{ edge->get_source() };

    const std::shared_ptr<rrt_star_node_t> child{ _tree.get_vertex_as<rrt_star_node_t>(child_idx) };
    const std::shared_ptr<rrt_star_node_t> parent{ _tree.get_vertex_as<rrt_star_node_t>(parent_idx) };

    for (auto state : *(edge->traj))
    {
      ofs_trajs << edge_idx << separating_value;
      ofs_trajs << state << "\n";
    }
    ofs_trajs << "\n";
  }

  ofs_trajs.close();
}
}  // namespace prx
