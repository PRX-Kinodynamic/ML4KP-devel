#pragma once

#include <queue>

#include "prx/planning/planners/planner.hpp"
#include "prx/planning/planner_functions/planner_functions.hpp"
#include "prx/utilities/data_structures/gnn.hpp"
#include "prx/utilities/data_structures/tree.hpp"
#include "prx/utilities/data_structures/sigma.hpp"
#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/timer.hpp"

#include "prx/simulation/observer.hpp"

namespace prx
{

class rrt_star_node_t : public tree_node_t
{
public:
  rrt_star_node_t()
  {
    cost_to_come = 0;
  }
  virtual ~rrt_star_node_t()
  {
  }

  friend std::ostream& operator<<(std::ostream& os, const rrt_star_node_t* obj)
  {
    os << *obj;
    return os;
  }
  friend std::ostream& operator<<(std::ostream& os, const std::shared_ptr<rrt_star_node_t> obj)
  {
    os << *obj;
    return os;
  }

  friend std::ostream& operator<<(std::ostream& os, const rrt_star_node_t& obj)
  {
    os << static_cast<const tree_node_t&>(obj) << prx::constants::separating_value;
    os << "cost_to_come" << prx::constants::separating_value;
    os << obj.cost_to_come << prx::constants::separating_value;
    return os;
  }
  double cost_to_come;
};

class rrt_star_edge_t : public tree_edge_t
{
public:
  rrt_star_edge_t()
  {
    edge_cost = 0;
  }
  virtual ~rrt_star_edge_t()
  {
  }

  std::shared_ptr<trajectory_t> traj;
  double edge_cost;
};

class rrt_star_specification_t : public planner_specification_t
{
public:
  rrt_star_specification_t(std::shared_ptr<system_group_t> sg, std::shared_ptr<collision_group_t> cg)
    : system_group(sg), state_space(sg->get_state_space()), eta_min(1), eta_max(10)
  {
    distance_function = [](const space_point_t& s1, const space_point_t& s2) { return space_t::euclidean_2d(s1, s2); };
    sample_state = [this](space_point_t& s) { default_sample_state(s, state_space); };
    valid_state = [this, cg](space_point_t& s) { return default_valid_state(s, state_space, cg); };
    valid_check = [&](trajectory_t& traj) { return default_valid_trajectory(traj, valid_state); };
    steer_function = [&](trajectory_t& traj, const space_point_t x_nearest, const space_point_t x_rand,
                         const double max_distance) {
      system_group->steer(traj, x_nearest, x_rand, max_distance, distance_function);
    };
  }
  virtual ~rrt_star_specification_t()
  {
  }

  cost_function_t cost_function;
  distance_function_t distance_function;
  sample_state_t sample_state;
  valid_trajectory_t valid_check;
  valid_state_t valid_state;
  steer_function_t steer_function;

  std::shared_ptr<system_group_t> system_group;

  space_t* state_space;

  // double eta;
  double eta_min;
  double eta_max;
};

class rrt_star_query_t : public planner_query_t
{
public:
  rrt_star_query_t(space_t* state_space, space_t* control_space) : planner_query_t(state_space, control_space)
  {
    clear_outputs();

    goal_region_radius = 0.5;
    start_state = state_space->make_point();
    goal_state = state_space->make_point();
    goal_check = [&](space_point_t s) { return default_goal_check(s, goal_state, goal_region_radius); };
  }
  virtual ~rrt_star_query_t()
  {
  }
  double goal_region_radius;
};

class rrt_star_t : public planner_t
{
public:
  using CloseNodes = std::vector<proximity_node_t*>;
  using NodePtr = std::shared_ptr<rrt_star_node_t>;

  rrt_star_t(const std::string& new_name);
  virtual ~rrt_star_t();

  virtual void print_statistics();

  virtual std::vector<std::string> get_statistics_header() override;
  virtual std::vector<double> get_statistics() override;
  virtual void to_files(const std::string file_prefix, const std::string directory = prx::out_path);
  virtual void from_files(const std::string file_prefix, const std::string directory);

  void connect_goal()
  {
    const double eta_min_copy{ _eta_min };
    sample_state_t sample_state_copy{ _sample_state };

    _eta_min = _eta_max;
    _sample_state = [&](space_point_t& s) { _state_space->copy(s, _goal_state); };

    condition_check_t check_one_iteration("iterations", 1);
    resolve_query(&check_one_iteration);

    _eta_min = eta_min_copy;
    _sample_state = sample_state_copy;
  }

protected:
  virtual void update_goal(const node_index_t node_index);

  void add_to_tree(NodePtr&, space_point_t);
  CloseNodes get_near_nodes(NodePtr&, const std::size_t);
  double connect_along_minimum_cost(rrt_star_node_t*&, NodePtr&, const CloseNodes&, trajectory_t&);
  void add_edge(const NodePtr, rrt_star_node_t*, trajectory_t&, const double);
  void rewire_tree(const CloseNodes&, NodePtr&);

  virtual void _link_and_setup_spec(planner_specification_t* spec) override;
  virtual bool _preprocess() override;
  virtual bool _link_and_setup_query(planner_query_t* query) override;
  virtual void _resolve_query(condition_check_t* condition) override;
  virtual void _fulfill_query() override;
  virtual void _reset() override;

  rrt_star_specification_t* _rrt_star_spec;
  rrt_star_query_t* _rrt_star_query;

  node_index_t _start_vertex;
  node_index_t _goal_vertex;

  distance_function_t _distance_function;
  cost_function_t _cost_function;
  sample_state_t _sample_state;
  valid_trajectory_t _valid_check;
  steer_function_t _steer_function;

  tree_t _tree;
  graph_nearest_neighbors_t* _metric;

  space_t* _state_space;

  space_point_t _x_rand;
  space_point_t _x_new;
  space_point_t _goal_state;
  double _eta;
  double _eta_min;
  double _eta_max;

  std::size_t _iteration_count;
  timer_t _timer;

  double _current_solution;
  double _current_solution_time;
  std::size_t _current_solution_iters;

  std::size_t _print_statistics_count;

  simulation::observer_t _observer;
  double _goal_region_radius;
  int k_RRT;
};
}  // namespace prx
