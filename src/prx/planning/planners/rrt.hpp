#pragma once

#include <stack>
#include <queue>

#include "prx/planning/planners/planner.hpp"
#include "prx/planning/planner_functions/planner_functions.hpp"
#include "prx/utilities/data_structures/gnn.hpp"
#include "prx/utilities/data_structures/tree.hpp"
#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/timer.hpp"
#include "prx/utilities/data_structures/sigma.hpp"

#include "prx/simulation/observer.hpp"

#define PLANNER_NAME "RRT"
namespace prx
{

class rrt_node_t : public tree_node_t
{
public:
  rrt_node_t()
  {
    cost_to_come = 0;
    duration = 0;
  }
  virtual ~rrt_node_t()
  {
  }

  double cost() const
  {
    return cost_to_come;
  }

  double& cost()
  {
    return cost_to_come;
  }

  virtual void copy(const rrt_node_t& other)
  {
    cost_to_come = other.cost_to_come;
    duration = other.duration;
  }

  template <typename NodePtr, typename EdgePtr>
  void update(NodePtr& parent_node, EdgePtr& parent_edge)
  {
    prx_assert(parent_node != nullptr, "Parent node is nullptr!");
    prx_assert(parent_edge != nullptr, "Parent node is nullptr!");
    cost_to_come = parent_node->cost_to_come + parent_edge->edge_cost;
    duration = parent_node->duration + parent_edge->plan->duration();
  }

  double cost_to_come;
  double duration;
};

class rrt_edge_t : public tree_edge_t
{
public:
  rrt_edge_t() : edge_cost(0)
  {
  }
  virtual ~rrt_edge_t()
  {
  }

  virtual void copy(const rrt_edge_t& other)
  {
    if (other.plan != nullptr)
    {
      plan = std::make_shared<plan_t>(*(other.plan));
    }
    if (other.traj != nullptr)
    {
      traj = std::make_shared<trajectory_t>(*(other.traj));
    }
    edge_cost = other.edge_cost;
  }

  void update(prx::plan_t& plan_in, prx::trajectory_t& traj_in, const double& cost)
  {
    plan = std::make_shared<prx::plan_t>(plan_in);
    traj = std::make_shared<prx::trajectory_t>(traj_in);
    edge_cost = cost;
  }

  friend std::ostream& operator<<(std::ostream& os, const std::shared_ptr<rrt_edge_t>& obj)
  {
    os << *obj;
    return os;
  }

  friend std::ostream& operator<<(std::ostream& os, const rrt_edge_t& obj)
  {
    os << static_cast<tree_edge_t>(obj) << prx::constants::separating_value;
    os << obj.plan->duration();

    return os;
  }

  std::shared_ptr<plan_t> plan;
  std::shared_ptr<trajectory_t> traj;
  double edge_cost;
};

class rrt_specification_t : public planner_specification_t
{
public:
  rrt_specification_t(std::shared_ptr<system_group_t> sg, std::shared_ptr<collision_group_t> cg)
  {
    _sg = sg;
    bnb = true;
    use_replanning = false;
    state_space = sg->get_state_space();
    control_space = sg->get_control_space();
    double multiplier = simulation_step >= 1 ? simulation_step : 1. / simulation_step;
    min_control_steps = 0.5 * multiplier;
    max_control_steps = 5 * multiplier;

    cost_function = [](const trajectory_t& t, const plan_t& plan) { return default_cost_function(t, plan); };
    distance_function = [](const space_point_t& s1, const space_point_t& s2) { return space_t::euclidean_2d(s1, s2); };
    sample_state = [this](space_point_t& s) { default_sample_state(s, state_space); };
    sample_plan = [this](plan_t& p, space_point_t pose) {
      // default_sample_plan(p,control_space,100,400);
      // Changed to time
      default_sample_plan(p, control_space, min_control_steps, max_control_steps);
    };
    valid_state = [this, cg](space_point_t& s) { return default_valid_state(s, state_space, cg); };
    valid_check = [&](trajectory_t& traj) { return default_valid_trajectory(traj, valid_state); };
    valid_stop_check = [this, sg, cg](space_point_t start_state, plan_t* stopping_plan, trajectory_t* stopping_traj) {
      return default_valid_stop(start_state, stopping_plan, stopping_traj, sg, cg);
    };
    propagate = [sg](space_point_t& start_state, plan_t& plan, trajectory_t& out_traj) {
      default_propagate(start_state, plan, out_traj, sg);
    };
    expand = [sg, this](space_point_t& s, std::vector<plan_t*>& plans, std::vector<trajectory_t*>& trajs, int bn,
                        bool blossom_expand) { default_expand(s, plans, trajs, bn, sg, sample_plan, propagate); };

    blossom_number = 1;
  }
  virtual ~rrt_specification_t()
  {
  }
  virtual void init(const prx::param_loader& params) override
  {
    bnb = params.exists("bnb") ? params["bnb"].as<bool>() : false;
    use_replanning = params.exists("use_replanning") ? params["use_replanning"].as<bool>() : false;

    if (params.exists("control_steps"))
    {
      const prx::param_loader params_cs{ params["control_steps"] };
      min_control_steps = params_cs.exists("min") ? params_cs["min"].as<int>() : min_control_steps;
      max_control_steps = params_cs.exists("max") ? params_cs["max"].as<int>() : max_control_steps;
    }
    blossom_number = params.exists("blossom_number") ? params["blossom_number"].as<int>() : blossom_number;
  }
  friend std::ostream& operator<<(std::ostream& os, const rrt_specification_t& obj)
  {
    os << "bnb: " << obj.bnb << "\n";
    os << "use_replanning: " << obj.use_replanning << "\n";
    os << "min_control_steps: " << obj.min_control_steps << "\n";
    os << "max_control_steps: " << obj.max_control_steps << "\n";
    os << "blossom_number: " << obj.blossom_number << "\n";

    return os;
  }
  std::shared_ptr<system_group_t> _sg;

  cost_function_t cost_function;
  distance_function_t distance_function;
  sample_state_t sample_state;
  sample_plan_t sample_plan;
  valid_trajectory_t valid_check;
  valid_stop_t valid_stop_check;
  valid_state_t valid_state;
  expand_t expand;
  propagate_t propagate;

  space_t* state_space;
  space_t* control_space;

  bool bnb;
  bool use_replanning;

  int min_control_steps;
  int max_control_steps;
  int blossom_number;
};

class rrt_query_t : public planner_query_t
{
public:
  rrt_query_t(space_t* state_space, space_t* control_space)
    : planner_query_t(state_space, control_space), total_solutions(1), goal_region_radius(0.5)
  {
    clear_outputs();

    goal_check = [&](space_point_t s) { return default_goal_check(s, goal_state, goal_region_radius); };
  }
  virtual ~rrt_query_t()
  {
  }

  virtual void init(const prx::param_loader& params) override
  {
    planner_query_t::init(params);
    if (goal_state and params.exists("goal"))
    {
      const prx::param_loader params_goal{ params["goal"] };
      goal_region_radius = params_goal.exists("radius") ? params_goal["radius"].as<double>() : goal_region_radius;
      total_solutions =
          params_goal.exists("total_solutions") ? params_goal["total_solutions"].as<int>() : total_solutions;
    }
  }

  friend std::ostream& operator<<(std::ostream& os, const rrt_query_t& obj)
  {
    os << static_cast<planner_query_t>(obj) << "\n";
    os << "goal_region_radius: " << obj.goal_region_radius;
    os << "total_solutions: " << obj.total_solutions;

    return os;
  }

  double goal_region_radius;
  int total_solutions;
};

class rrt_t : public planner_t
{
public:
  using Node = rrt_node_t;
  using Edge = rrt_edge_t;
  using EdgePtr = std::shared_ptr<Edge>;
  using NodePtr = std::shared_ptr<Node>;

  rrt_t(const std::string& new_name);
  virtual ~rrt_t();

  virtual void print_statistics();

  virtual std::vector<std::string> get_statistics_header() override;
  virtual std::vector<double> get_statistics() override;

  graph_nearest_neighbors_t* graph_nearest_neighbors() const
  {
    return metric;
  }

  tree_t& tree()
  {
    return _tree;
  }

  const tree_t& tree() const
  {
    return _tree;
  }

  template <typename RootNode>
  std::shared_ptr<RootNode> root() const
  {
    return _tree.get_vertex_as<RootNode>(start_vertex);
  }

  virtual std::shared_ptr<prx::tree_t> tree_of_solutions()
  {
    // const double radius{ rrt_query->goal_region_radius };
    const space_point_t goal_state{ rrt_query->goal_state };
    const int total_solutions{ rrt_query->total_solutions };

    // const space_point_t& goal_state{ rrt_query_t->goal_state };
    const std::vector<prx::proximity_node_t*> goal_nodes{ metric->multi_query(goal_state, total_solutions) };

    // std::shared_ptr<Node> root{ _tree.get_vertex_as<Node>(goal_nodes) };

    return _tree_of_solutions<Node, Edge>(goal_nodes);
  }

  // void split_edge(EdgePtr old_edge, EdgePtr new_edge, NodePtr source_node, NodePtr new_target,
  //                 const double time_of_split)
  EdgePtr split_edge(EdgePtr e0, const double time_of_split)
  {
    EdgePtr e1{ _tree.split_edge<Node, Edge>(e0->get_index()) };

    NodePtr n0{ _tree.get_vertex_as<Node>(e0->get_source()) };
    NodePtr n1{ _tree.get_vertex_as<Node>(e1->get_target()) };
    NodePtr n2{ _tree.get_vertex_as<Node>(e1->get_source()) };

    prx::plan_t e1_plan{ e0->plan->split(time_of_split) };
    prx::trajectory_t e1_traj{ e0->traj->split(time_of_split) };
    double e1_cost{ cost_function(e1_traj, e1_plan) };

    e1->update(e1_plan, e1_traj, e1_cost);
    n2->point = state_space->clone_point(e0->traj->back());
    n2->update(n0, e0);
    return e1;
  }

  template <typename Edge>
  static void tree_to_trajectories(prx::tree_t& tree, std::vector<trajectory_t>& trajectories)
  {
    auto iter_bounds = tree.edges();
    for (auto iter = iter_bounds.first; iter != iter_bounds.second; iter++)
    {
      trajectories.push_back(*tree.get_edge_as<Edge>((*iter)->get_index())->traj);
    }
  }

  template <typename Edge>
  static std::vector<trajectory_t> tree_to_trajectories(prx::tree_t& tree)
  {
    std::vector<trajectory_t> trajs;
    rrt_t::tree_to_trajectories<Edge>(tree, trajs);
    return trajs;
  }

protected:
  virtual void update_goal(node_index_t node_index);

  virtual void _link_and_setup_spec(planner_specification_t* spec) override;
  virtual bool _preprocess() override;
  virtual bool _link_and_setup_query(planner_query_t* query) override;
  virtual void _resolve_query(condition_check_t* condition) override;
  virtual void _fulfill_query() override;
  virtual void _reset() override;

  node_index_t add_to_tree(prx::trajectory_t& traj, rrt_node_t* closest_node, const double& edge_cost,
                           prx::plan_t& plan);

  template <typename Node, typename Edge>
  std::shared_ptr<prx::tree_t> _tree_of_solutions(const std::vector<prx::proximity_node_t*> goal_nodes)
  {
    std::stack<Node*> solution_nodes;
    std::unordered_set<prx::node_index_t> visited;
    std::queue<prx::proximity_node_t*> to_visit{};

    for (auto node : goal_nodes)
    {
      // Node* curr_node{ dynamic_cast<Node*>(node) };

      to_visit.push(node);
    }

    std::shared_ptr<Node> root{ _tree.get_vertex_as<Node>(start_vertex) };
    visited.insert(root->get_index());
    // [ original_index ] -> new_index
    std::map<prx::node_index_t, prx::node_index_t> new_index_map;
    std::shared_ptr<prx::tree_t> sln_tree{ std::make_shared<prx::tree_t>() };

    // Add the root to the tree
    const prx::node_index_t start_vertex{ sln_tree->add_vertex<Node, Edge>() };
    std::shared_ptr<Node> new_root_node{ sln_tree->get_vertex_as<Node>(start_vertex) };
    new_root_node->point = state_space->make_point();
    state_space->copy(new_root_node->point, root->point);
    new_root_node->copy(*root);

    new_index_map[root->get_index()] = start_vertex;

    while (to_visit.size() > 0)
    {
      Node* curr_node{ dynamic_cast<Node*>(to_visit.front()) };
      const prx::node_index_t parent{ curr_node->get_parent() };

      if (visited.count(parent) == 0)
      {
        to_visit.push(_tree[parent].get());
        visited.insert(curr_node->get_index());
      }
      solution_nodes.push(curr_node);
      to_visit.pop();
    }
    // solution_nodes.push(root);

    while (not solution_nodes.empty())
    {
      Node* node{ solution_nodes.top() };
      // add node
      const prx::node_index_t new_node_index{ sln_tree->add_vertex<Node, Edge>() };
      new_index_map[node->get_index()] = new_node_index;
      std::shared_ptr<Node> new_tree_node{ sln_tree->get_vertex_as<Node>(new_node_index) };
      new_tree_node->point = state_space->make_point();
      state_space->copy(new_tree_node->point, node->point);
      new_tree_node->copy(*node);
      solution_nodes.pop();
    }

    // new_index_map[node->get_index()]
    for (auto pair : new_index_map)
    {
      const node_index_t old_index{ pair.first };
      const node_index_t new_index{ pair.second };
      std::shared_ptr<Node> old_node{ _tree.get_vertex_as<Node>(old_index) };

      const node_index_t old_parent{ old_node->get_parent() };
      if (old_index == old_parent)
        continue;
      const prx::node_index_t parent_index{ new_index_map[old_parent] };
      const std::shared_ptr<Edge> old_edge{ _tree.get_edge_as<Edge>(old_node->get_parent_edge()) };

      // const prx::node_index_t new_node_index{ new_index_map[node->get_index()] };

      const prx::edge_index_t edge_index{ sln_tree->add_edge(parent_index, new_index) };
      std::shared_ptr<Edge> new_edge{ sln_tree->get_edge_as<Edge>(edge_index) };
      new_edge->copy(*old_edge);
    }
    // PRX_DEBUG_VAR_1(sln_tree->size());
    return sln_tree;
  }

  virtual void bnb(node_index_t v, double cost_bound, bool delete_flag = false);

  rrt_specification_t* rrt_spec;
  rrt_query_t* rrt_query;

  std::string planner_name;

  node_index_t start_vertex;
  node_index_t goal_vertex;

  distance_function_t distance_function;
  cost_function_t cost_function;
  sample_state_t sample_state;
  sample_plan_t sample_plan;
  valid_trajectory_t valid_check;
  valid_stop_t valid_stop_check;
  expand_t expand;
  propagate_t propagate;

  tree_t _tree;
  graph_nearest_neighbors_t* metric;

  space_t* state_space;
  space_t* control_space;

  space_point_t sample_point;

  long unsigned iteration_count;
  timer_t timer;

  double current_solution;
  long unsigned current_solution_iters;
  double current_solution_time;

  bool use_replanning;

  int print_statistics_count;

  simulation::observer_t observer;

  bool _bnb;
};
}  // namespace prx
