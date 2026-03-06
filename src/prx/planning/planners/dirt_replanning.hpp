#pragma once

#include <iterator>
#include "prx/planning/planners/dirt.hpp"
#include "prx/planning/planners/rrt.hpp"
#include "prx/utilities/general/time_profiler.hpp"

namespace prx
{

class dirt_replan_node_t : public rrt_node_t
{
public:
  dirt_replan_node_t() : rrt_node_t()
  {
    bridge = false;
    is_blossom_expand_done = false;
    random_expand = false;
    is_safe = false;
    safety_time = 0;
  }
  virtual ~dirt_replan_node_t()
  {
    for (auto eg : edge_generators)
    {
      if (eg.first != nullptr)
        delete eg.first;
      if (eg.second != nullptr)
        delete eg.second;
    }
    edge_generators.clear();
    is_blossom_expand_done = false;
    random_expand = false;
  }

  double cost_to_go;

  bool bridge;

  double dir_radius;

  int blossom_number;

  bool is_blossom_expand_done, random_expand;

  std::vector<std::pair<plan_t*, trajectory_t*>> edge_generators;

  std::vector<int> indices;

  double checkpoint_time, safety_time;

  bool is_safe;
};

class dirt_replan_specification_t : public rrt_specification_t
{
public:
  dirt_replan_specification_t(std::shared_ptr<system_group_t> sg, std::shared_ptr<collision_group_t> cg)
    : rrt_specification_t(sg, cg)
    , profile(false)
    , output_path(out_path)
    , use_pruning(true)
    , use_contingency(true)
    , planning_cycle_duration(1.0)
  {
    blossom_number = 5;
    f_function = [this](const double& g, const double& h) { return default_f_value_function(g, h); };

    heuristic = [this](const space_point_t& s, const space_point_t& s2) {
      return default_heuristic_function(s, s2, distance_function);
    };
    wavefront_h = heuristic;
    contingency_check = [&](trajectory_t& traj) { return default_valid_trajectory(traj, valid_state); };
    plan_safety_check = [&](trajectory_t& traj) { return default_valid_trajectory(traj, valid_state); };
  }
  virtual ~dirt_replan_specification_t()
  {
  }

  static prx::param_loader init()
  {
    prx::param_loader params{ rrt_specification_t::init() };

    params["planning_cycle_duration"].set(decltype(planning_cycle_duration){});
    params["use_pruning"].set(decltype(use_pruning){});
    params["use_contingency"].set(decltype(use_contingency){});
    params["profile"].set(decltype(profile){});
    params["output_path"].set(decltype(output_path){});
    return params;
  }

  virtual void init(const prx::param_loader& params) override
  {
    rrt_specification_t::init(params);

    SET_VARIABLE(params, planning_cycle_duration)
    SET_VARIABLE(params, use_pruning)
    SET_VARIABLE(params, use_contingency)
    SET_VARIABLE(params, profile)
    SET_VARIABLE(params, output_path)
  }

  // int blossom_number;
  double planning_cycle_duration;

  bool use_pruning, use_contingency;
  f_value_function_t f_function;
  heuristic_function_t heuristic, wavefront_h;
  valid_trajectory_t contingency_check;
  valid_trajectory_t plan_safety_check;

  bool profile;
  std::string output_path;
};

class dirt_replan_query_t : public rrt_query_t
{
public:
  enum solution_type_t
  {
    TREE_TRAJECTORY = 0,
    WAVEFRONT,
    NONE
  };

  dirt_replan_query_t(space_t* state_space, space_t* control_space)
    : rrt_query_t(state_space, control_space), _sln_type(solution_type_t::TREE_TRAJECTORY)
  {
    start_time = 0.0;
  }

  virtual ~dirt_replan_query_t()
  {
  }

  static prx::param_loader init()
  {
    prx::param_loader params{ rrt_query_t::init() };
    params["solution_type"].set("TREE_TRAJECTORY | WAVEFRONT");

    return params;
  }

  virtual void init(const prx::param_loader& params) override
  {
    rrt_query_t::init(params);

    if (params.exists("solution_type"))
    {
      const std::string sln_type{ params["solution_type"].as<>() };
      if (sln_type == "TREE_TRAJECTORY")
      {
        _sln_type = solution_type_t::TREE_TRAJECTORY;
      }
      else if (sln_type == "WAVEFRONT")
      {
        _sln_type = solution_type_t::WAVEFRONT;
      }
      else
      {
        prx_throw("[dirt_replan_query_t::init] invalid solution_type: {TREE_TRAJECTORY, WAVEFRONT}")
      }
    }
  }

  friend std::ostream& operator<<(std::ostream& os, const dirt_replan_query_t& obj)
  {
    std::string sln_str{ "??" };
    if (obj._sln_type == solution_type_t::WAVEFRONT)
    {
      sln_str = "WAVEFRONT";
    }
    else if (obj._sln_type == solution_type_t::TREE_TRAJECTORY)
    {
      sln_str = "TREE_TRAJECTORY";
    }

    os << static_cast<rrt_query_t>(obj);
    os << "start_time: " << obj.start_time << "\n";
    // os << "previous_contingency: " << obj.previous_contingency << "\n";

    os << "solution_type: " << sln_str << "\n";

    return os;
  }

  double start_time;
  // bool previous_contingency;

  solution_type_t _sln_type;
};

class dirt_replan_t : public rrt_t
{
public:
  using Node = dirt_replan_node_t;
  using Edge = rrt_edge_t;
  using EdgePtr = std::shared_ptr<Edge>;
  using NodePtr = std::shared_ptr<Node>;
  using SolutionType = dirt_replan_query_t::solution_type_t;
  struct dirt_counter_t
  {
    dirt_counter_t() : bnb(0), pruning(0), collision_check(0), accepted(0), f_rejected(0) {};
    void reset()
    {
      f_rejected = 0;
      pruning = 0;
      collision_check = 0;
      bnb = 0;
      accepted = 0;
    }
    std::size_t f_rejected;       // New nodes not added due to f(new_node) > f(current_sln)
    std::size_t pruning;          // New nodes not added due to dirt-pruning (using dir_radius)
    std::size_t collision_check;  // New nodes not added due to being in collision
    std::size_t accepted;         // New nodes accepted to be added to the tree (could be removed later)
    std::size_t bnb;              // Nodes deleted in a BNB operation when a new solution is found
  };

  struct statistics_t : public planner_t::statistics_t
  {
    statistics_t(const planner_t::statistics_t& planner_stats_, const dirt_counter_t random_edges_counter_,
                 const dirt_counter_t blossom_edges_counter_, const SolutionType solution_type_)
      : planner_t::statistics_t(planner_stats_)
      , random_edges_counter(random_edges_counter_)
      , blossom_edges_counter(blossom_edges_counter_)
      , solution_type(solution_type_)
    {
    }
    virtual ~statistics_t() {};

    const dirt_counter_t random_edges_counter;
    const dirt_counter_t blossom_edges_counter;
    const SolutionType solution_type;
  };

  dirt_replan_t(const std::string& new_name);
  virtual ~dirt_replan_t();

  dirt_counter_t _random_edges_counter, _blossom_edges_counter;

  node_index_t get_best_node_index()
  {
    return best_node;
  }

  virtual planner_t::statistics_t statistics() override
  {
    return statistics_t(rrt_t::statistics(),     // no-lint
                        _random_edges_counter,   // no-lint
                        _blossom_edges_counter,  // no-lint
                        _current_solution_type);
  }

protected:
  virtual void update_goal(node_index_t node_index) override;

  virtual void _link_and_setup_spec(planner_specification_t* spec) override;
  virtual bool _preprocess() override;
  virtual bool _link_and_setup_query(planner_query_t* query) override;
  virtual void _resolve_query(condition_check_t* condition) override;
  virtual void _fulfill_query() override;
  virtual void _reset() override;

  virtual std::vector<double> get_statistics() override;

  dirt_replan_specification_t* dirt_spec;
  dirt_replan_query_t* dirt_replan_query;

  trajectory_t* stopping_traj;
  plan_t* stopping_plan;
  space_point_t last_safe_state;

  virtual void bnb(node_index_t v, double cost_bound, bool delete_flag = false) override;

private:
  std::shared_ptr<time_profiler_t> _resolve_profiler;
  std::shared_ptr<time_profiler_t> _fulfill_profiler;

  dirt_replan_query_t::solution_type_t _current_solution_type;
  f_value_function_t _f_function;
  heuristic_function_t _heuristic, wavefront_h;
  expand_t expand;
  valid_trajectory_t contingency_check;
  valid_trajectory_t plan_safety_check;

  double planning_cycle_duration;
  double multiplier;
  node_index_t best_node;
  double best_cost;

  double max_radius;
  bool child_extension;
  node_index_t previous_child;

  bool tree_solution();
  bool wavefront_solution();

  void add_edge_to_tree(std::pair<plan_t*, trajectory_t*> eg, dirt_replan_node_t* closest_node,
                        std::vector<dirt_replan_node_t*> dir_updates, double new_node_dir_radius);

  dirt_replan_node_t* get_vertex(node_index_t v) const
  {
    return tree().get_vertex_as<dirt_replan_node_t>(v).get();
  }

  bool is_leaf(node_index_t v)
  {
    return (get_vertex(v)->get_children().empty());
  }

  void remove_leaf(node_index_t v)
  {
    prx_assert(is_leaf(v), "Trying to remove a tree node that is not a leaf!");

    if (!get_vertex(v)->bridge)
    {
      metric->remove_node(get_vertex(v));
      get_vertex(v)->bridge = true;
    }
    tree().remove_vertex(v);
  }

  bool is_best_goal(node_index_t v) const
  {
    node_index_t new_v = goal_vertex;
    while (get_vertex(new_v)->get_parent() != new_v)
    {
      if (new_v == v)
        return true;
      new_v = get_vertex(new_v)->get_parent();
    }
    return false;
  }
};
}  // namespace prx
