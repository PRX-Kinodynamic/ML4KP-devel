#pragma once

#include <iterator>
#include <memory>
#include <queue>
#include <string>

namespace prx
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

template <typename StateSpace, typename Node, typename NearestNeighbors>
class planner_memory_t
{
public:
  using StateSpacePtr = std::shared_ptr<StateSpace>;
  using NearestNeighborsPtr = std::shared_ptr<NearestNeighbors>;

  planner_memory_t() {};

  StateSpacePtr state_space()
  {
    return _state_space;
  }

  NearestNeighborsPtr nearest_neighbors()
  {
    return _nearest_neighbors;
  }

protected:
  StateSpacePtr _state_space;
  NearestNeighborsPtr _nearest_neighbors;
};

// Example of PlannerFunctions: The functions need to exist to use replanner_t but
// it is not required to derive from this specific class.
template <typename PlannerSpec, typename PlannerQuery, typename PlannerMemory>
class planner_functions_t
{
public:
  using PlannerSpecPtr = std::shared_ptr<PlannerSpec>;
  using PlannerQueryPtr = std::shared_ptr<PlannerQuery>;
  using PlannerMemoryPtr = std::shared_ptr<PlannerMemory>;
  virtual void set_specification(const PlannerSpecPtr spec)
  {
    _planner_spec = std::make_shared<PlannerSpec>(*spec);
  }
  virtual void set_query(PlannerQueryPtr query)
  {
    _planner_query = query;
  }
  virtual void condition_check(PlannerMemoryPtr) = 0;
  virtual void node_selection(PlannerMemoryPtr) = 0;
  virtual void expand(PlannerMemoryPtr) = 0;
  virtual void node_validation(PlannerMemoryPtr) = 0;
  virtual void update_graph(PlannerMemoryPtr) = 0;
  virtual void update_solution(PlannerMemoryPtr) = 0;
  virtual void update_stats(PlannerMemoryPtr) = 0;
  virtual void answer_query(PlannerMemoryPtr) = 0;
  virtual void postprocess(PlannerMemoryPtr) = 0;
  virtual void reset(PlannerMemoryPtr) = 0;

protected:
  PlannerSpecPtr _planner_spec;
  PlannerQueryPtr _planner_query;
};

template <typename PlannerFunctions, typename PlannerMemory>
class motion_planner_t
{
public:
  using PlannerFunctionsPtr = std::shared_ptr<PlannerFunctions>;
  using PlannerMemoryPtr = std::shared_ptr<PlannerMemory>;

  enum stage_t
  {
    IDLE = 0,
    SET_SPECIFICATION,
    PREPROCESS,
    SET_QUERY,
    PLAN,  // Resolve query
    ANSWER_QUERY,
    POSTPROCESS
  };

  motion_planner_t(const std::string planner_name) : _planner_name(planner_name), _current_stage(stage_t::IDLE)
  {
    _planner_memory = std::make_shared<PlannerMemory>();
    _planner_functions = std::make_shared<PlannerFunctionsPtr>();
  }

  virtual ~motion_planner_t() {};

  template <typename PlannerSpec>
  void set_specification(const std::shared_ptr<PlannerSpec> spec)
  {
    check_stage(stage_t::IDLE, stage_t::POSTPROCESS);

    // _planner_query
    _planner_functions->set_specification(spec, _planner_memory);
    _current_stage = stage_t::SET_SPECIFICATION;
  }

  void preprocess()
  {
    check_stage(stage_t::SET_SPECIFICATION);

    _current_stage = stage_t::PREPROCESS;
  }

  template <typename PlannerQuery>
  void set_query(std::shared_ptr<PlannerQuery> query)
  {
    check_stage(stage_t::PREPROCESS);
    _planner_functions->set_query(query);
    _current_stage = stage_t::SET_QUERY;
  }

  void plan()
  {
    check_stage(stage_t::SET_QUERY);
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

  void answer_query()
  {
    check_stage(stage_t::PLAN);

    _planner_functions->answer_query(_planner_memory);
    _current_stage = stage_t::POSTPROCESS;
  }

  void postprocess()
  {
    check_stage(stage_t::POSTPROCESS);

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
}  // namespace prx
// #include <prx/planning/planners/replanner-inl.hpp>
