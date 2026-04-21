#pragma once

#include <iterator>
#include <memory>
#include <queue>
#include <string>
#include "loaders/obstacle_loader.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/planning/planners/planner_statistics.hpp"

namespace prx
{
namespace planners
{
template <typename SpacePoint>
class node_t
{
public:
  SpacePoint state()
  {
    return _state;
  }

protected:
  SpacePoint _state;
};

template <typename Derived>
struct planner_specification_t
{
  // using System = typename Derived::DynamicalSystem;
};

template <typename Derived>
class planner_memory_t
{
public:
  using System = typename planner_specification_t<Derived>::System;
  using SystemPtr = std::shared_ptr<System>;
  using State = typename System::State;
  using StateSpace = typename System::StateSpace;
  using StateSpacePtr = std::shared_ptr<StateSpace>;
  using NearestNeighbors = typename planner_specification_t<Derived>::NearestNeighbors;
  using NearestNeighborsPtr = std::shared_ptr<NearestNeighbors>;
  using CollisionChecker = typename planner_specification_t<Derived>::CollisionChecker;
  using CollisionCheckerPtr = std::shared_ptr<CollisionChecker>;
  using GoalChecker = typename planner_specification_t<Derived>::GoalChecker;

  using Node = typename planner_specification_t<Derived>::Node;
  using NodePtr = std::shared_ptr<Node>;
  using Tree = typename planner_specification_t<Derived>::Tree;
  using TreePtr = std::shared_ptr<Tree>;
  using TreeStatistics = prx::planner_statistics::tree_statistics_t;
  using TreeStatisticsPtr = std::shared_ptr<TreeStatistics>;

  planner_memory_t()
    : _system(std::make_shared<System>(System::default_params()))
    , _obstacles()
    , _nearest_neighbors(std::make_shared<NearestNeighbors>())
    , _collision_checker(system, _obstacles)
    , _tree(std::make_shared<Tree>())
    , _goal_checker()
    , _goal_node(nullptr)
    , _root_node(nullptr)
  {
  }

  planner_memory_t(prx::param_loader params, prx::param_loader environment)
    : _system(std::make_shared<System>(params["System"]))
    , _obstacles(environment)
    , _nearest_neighbors(std::make_shared<NearestNeighbors>(params["NearestNeighbors"]))
    , _collision_checker(std::make_shared<CollisionChecker>(_system, _obstacles))
    , _tree(std::make_shared<Tree>())
    , _goal_checker(params["GoalChecker"])
    , _goal_node(nullptr)
    , _root_node(nullptr)
  {
    _system->environment(_obstacles);
  }

  virtual void initialize(prx::param_loader) = 0;

  SystemPtr system()
  {
    return _system;
  }

  StateSpacePtr state_space()
  {
    return _system->state_space();
  }

  NearestNeighborsPtr nearest_neighbors()
  {
    return _nearest_neighbors;
  }

  TreeStatisticsPtr statistics()
  {
    return _stats;
  }

  CollisionCheckerPtr collision_checker()
  {
    return _collision_checker;
  }

  TreePtr tree()
  {
    return _tree;
  }

  bool goal_check(const State& state)
  {
    return _goal_checker(state);
  }

  void goal_node(const NodePtr node)
  {
    _goal_node = node;
  }

  NodePtr goal_node()
  {
    return _goal_node;
  }

  void root_node(const NodePtr node)
  {
    _root_node = node;
  }

  NodePtr root_node()
  {
    return _root_node;
  }

protected:
  // StateSpacePtr _state_space;
  SystemPtr _system;
  prx::obstacle_loader_t _obstacles;
  TreeStatisticsPtr _stats;
  NodePtr _goal_node;
  NodePtr _root_node;

  TreePtr _tree;
  NearestNeighborsPtr _nearest_neighbors;
  CollisionCheckerPtr _collision_checker;
  GoalChecker _goal_checker;
};

// Example of PlannerFunctions: The functions need to exist to use replanner_t but
// it is not required to derive from this specific class.
template <typename PlannerMemory>
class planner_functions_t
{
public:
  // using PlannerSpecPtr = std::shared_ptr<PlannerSpec>;
  // using PlannerQueryPtr = std::shared_ptr<PlannerQuery>;
  using PlannerMemoryPtr = std::shared_ptr<PlannerMemory>;

  planner_functions_t() {};

  planner_functions_t(prx::param_loader params)
  {
  }

  // SETUP
  virtual void initialize(prx::param_loader, PlannerMemoryPtr) = 0;
  virtual void preprocess(PlannerMemoryPtr) = 0;

  // PLANNING
  virtual bool condition_check(PlannerMemoryPtr) = 0;
  virtual void node_selection(PlannerMemoryPtr) = 0;
  virtual void expand(PlannerMemoryPtr) = 0;
  virtual void node_validation(PlannerMemoryPtr) = 0;
  virtual void update_graph(PlannerMemoryPtr) = 0;
  virtual void update_solution(PlannerMemoryPtr) = 0;
  virtual void update_stats(PlannerMemoryPtr) = 0;

  // AFTER PLANNING
  virtual void postprocess(PlannerMemoryPtr) = 0;
  virtual void reset(PlannerMemoryPtr) = 0;

protected:
  // PlannerSpecPtr _planner_spec;
  // PlannerQueryPtr _planner_query;
};

template <typename PlannerMemory, typename PlannerFunctions>
class motion_planner_t
{
public:
  using PlannerMemoryPtr = std::shared_ptr<PlannerMemory>;
  using PlannerFunctionsPtr = std::shared_ptr<PlannerFunctions>;

  enum stage_t
  {
    IDLE = 0,
    INITIALIZE,
    PREPROCESS,
    SET_QUERY,
    PLAN,  // Resolve query
    // ANSWER_QUERY,
    POSTPROCESS
  };

  motion_planner_t(const std::string planner_name)
    : _planner_name(planner_name)
    , _current_stage(stage_t::IDLE)
    , _planner_memory(std::make_shared<PlannerMemory>())
    , _planner_functions(std::make_shared<PlannerFunctions>())
  {
  }

  motion_planner_t(prx::param_loader params, prx::param_loader environment)
    : _current_stage(stage_t::IDLE)
    , _planner_name(params.get_or_default("name", std::string("MotionPlanner")))
    , _planner_memory(std::make_shared<PlannerMemory>(params["memory"], environment))
    , _planner_functions(std::make_shared<PlannerFunctions>(params["functions"]))
  {
    // _planner_memory = std::make_shared<PlannerMemory>(params["memory"]);
  }

  virtual ~motion_planner_t() {};

  void initialize(prx::param_loader params)
  {
    check_stage(stage_t::IDLE, stage_t::POSTPROCESS);

    // _planner_query
    _planner_functions->initialize(params, _planner_memory);
    _current_stage = stage_t::INITIALIZE;
  }

  void preprocess()
  {
    check_stage(stage_t::INITIALIZE);
    _planner_functions->preprocess(_planner_memory);

    _current_stage = stage_t::PREPROCESS;
  }

  // template <typename PlannerQuery>
  // void set_query(std::shared_ptr<PlannerQuery> query)
  // {
  //   check_stage(stage_t::PREPROCESS);
  //   _planner_functions->set_query(query);
  //   _current_stage = stage_t::SET_QUERY;
  // }

  void plan()
  {
    check_stage(stage_t::PREPROCESS);
    _current_stage = stage_t::PLAN;

    while (_planner_functions->condition_check(_planner_memory))
    {
      // Select the node to expand
      _planner_functions->node_selection(_planner_memory);
      // Expand the node(s)
      _planner_functions->expand(_planner_memory);
      // Validate node(s) (e.i. collision check, g-value, f-value, etc)
      _planner_functions->node_validation(_planner_memory);
      // Update the graph given the validated nodes
      _planner_functions->update_graph(_planner_memory);
      // Update the current best solution
      _planner_functions->update_solution(_planner_memory);
      // Update the planner statistics
      _planner_functions->update_stats(_planner_memory);
    }
    // std::static_pointer_cast<Planner>(this)->plan_impl();
  }

  // void answer_query()
  // {
  //   check_stage(stage_t::PLAN);

  //   _planner_functions->answer_query(_planner_memory);
  //   _current_stage = stage_t::POSTPROCESS;
  // }

  void postprocess()
  {
    check_stage(stage_t::PLAN);

    _planner_functions->postprocess(_planner_memory);
    _current_stage = stage_t::IDLE;
  }

  void reset()
  {
    _planner_functions->reset(_planner_memory);
    _current_stage = stage_t::IDLE;
  }

protected:
  // template <std::size_t I, typename... Tp, std::enable_if_t<(I == sizeof...(Tp) - 1), bool> = true>
  template <typename... Stages, std::enable_if_t<(0 == sizeof...(Stages)), bool> = true>
  void check_stage(const stage_t& expected, const Stages&... others)
  {
    if (_current_stage == expected)
      return;
    PRX_WARNING("[" << _planner_name << "]" << " is not in the correct stage. Resetting...");
    reset();
  }

  template <typename... Stages, std::enable_if_t<(1 >= sizeof...(Stages)), bool> = true>
  void check_stage(const stage_t& expected, const Stages&... others)
  {
    if (_current_stage == expected)
      return;
    check_stage(others...);
  }

  PlannerMemoryPtr _planner_memory;
  PlannerFunctionsPtr _planner_functions;

  std::string _planner_name;
  stage_t _current_stage;
};
}  // namespace planners
}  // namespace prx
// #include <prx/planning/planners/replanner-inl.hpp>
