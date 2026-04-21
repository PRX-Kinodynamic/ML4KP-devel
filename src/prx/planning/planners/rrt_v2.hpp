#pragma once

#include <memory>
#include <stack>
#include <queue>
#include <type_traits>

// #include "prx/planning/planners/planner.hpp"
// #include "prx/planning/planner_functions/planner_functions.hpp"
// #include "prx/utilities/data_structures/gnn.hpp"
// #include "prx/utilities/data_structures/tree.hpp"
// #include "prx/utilities/defs.hpp"
// #include "prx/utilities/general/timer.hpp"
// #include "prx/utilities/data_structures/sigma.hpp"

#include "general/param_loader.hpp"
#include "planners/rrt.hpp"
#include "playback/trajectory.hpp"
#include "prx/utilities/general/condition_check.hpp"
#include "prx/planning/planners/replanner.hpp"
#include "prx/planning/planners/node_selection.hpp"
#include "prx/planning/planners/node_expansion.hpp"
#include "prx/planning/planners/node_validation.hpp"
#include "prx/planning/planners/graph_update.hpp"
#include "prx/planning/planners/solution_update.hpp"
#include "prx/simulation/playback/piecewise_plan.hpp"
#include "prx/simulation/playback/trajectory_v2.hpp"
#include "prx/utilities/spaces/sampler.hpp"
#include "prx/simulation/forward_propagation.hpp"
#include "prx/simulation/collision_checking/pqp_collision_checker.hpp"
#include "prx/utilities/data_structures/abstract_tree.hpp"
#include "prx/planning/planner_functions/goal_checkers.hpp"

#include "prx/utilities/data_structures/graph_nearest_neighbors.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/simulation/playback/plan.hpp"
#include "prx/simulation/playback/trajectory.hpp"

#include "prx/planning/planners/replanner.hpp"

namespace prx
{
namespace planners
{
template <typename State>
struct rrt_node_t : public fast_tree_node_t
{
  void state(const State& state)
  {
    _state = state;
  }

  State& state()
  {
    return _state;
  }

  void cost_to_come(const double& new_cost)
  {
    _cost_to_come = new_cost;
  }

  double cost_to_come() const
  {
    return _cost_to_come;
  }

  State _state;
  double _cost_to_come;
};

template <typename Controller, typename Trajectory>
struct rrt_edge_t : public fast_tree_edge_t
{
  using ControllerPtr = std::shared_ptr<Controller>;
  using TrajectoryPtr = std::shared_ptr<Trajectory>;
  void controller(ControllerPtr controller)
  {
    _controller = controller;
  }
  void trajectory(TrajectoryPtr trajectory)
  {
    _trajectory = trajectory;
  }

  ControllerPtr controller()
  {
    return _controller;
  }
  TrajectoryPtr trajectory()
  {
    return _trajectory;
  }

  ControllerPtr _controller;
  TrajectoryPtr _trajectory;
};

// struct rrt_spec_t
// {
//   rrt_spec_t(prx::param_loader params)
//   {
//   }
// };

// struct rrt_query_t
// {
//   rrt_query_t(prx::param_loader params)
//   {
//   }
// };

template <typename DynamicalSystem>
class rrt_memory_t;

template <typename DynamicalSystem>
struct planner_specification_t<rrt_memory_t<DynamicalSystem>>
{
  using System = DynamicalSystem;
  using StateSpace = typename DynamicalSystem::StateSpace;
  using State = typename DynamicalSystem::State;
  using Control = typename DynamicalSystem::Control;
  using Controller = prx::experimental::piecewise_plan_t<Control, double>;
  using Trajectory = prx::experimental::trajectory_t<StateSpace, double>;

  using Node = rrt_node_t<State>;
  using NodePtr = std::shared_ptr<Node>;
  using Edge = rrt_edge_t<Controller, Trajectory>;
  using DistanceFunction = std::function<double(const NodePtr&, const NodePtr&)>;
  using NearestNeighbors = prx::data_structures::graph_nearest_neighbors_t<NodePtr, DistanceFunction>;
  using CollisionChecker = typename prx::collision_checking::pqp_checker_t<DynamicalSystem>;
  using GoalChecker = typename prx::goal_radius_checker_t<State>;
  using Tree = prx::fast_tree_t<Node, Edge>;

  using ForwardPropagation = prx::forward_propagation_t<System, Control>;
};

// class
template <typename DynamicalSystem>
class rrt_memory_t : public planner_memory_t<rrt_memory_t<DynamicalSystem>>
{
  using This = rrt_memory_t<DynamicalSystem>;

  using DistanceFunction = typename planner_specification_t<This>::DistanceFunction;
  using State = typename planner_specification_t<This>::State;
  using StateSpace = typename planner_specification_t<This>::StateSpace;

  using Node = typename planner_specification_t<This>::Node;
  using NodePtr = std::shared_ptr<Node>;

  using System = typename planner_specification_t<This>::System;
  using SystemPtr = std::shared_ptr<System>;
  using Controller = typename planner_specification_t<This>::Controller;
  using ControllerPtr = std::shared_ptr<Controller>;
  using Trajectory = typename planner_specification_t<This>::Trajectory;
  using TrajectoryPtr = std::shared_ptr<Trajectory>;
  using ForwardPropagation = typename planner_specification_t<This>::ForwardPropagation;

public:
  using Base = planner_memory_t<rrt_memory_t<DynamicalSystem>>;

  using NearestNeighborsPtr = typename Base::NearestNeighborsPtr;

  std::shared_ptr<prx::condition_check_t> condition;

  rrt_memory_t() : Base(), _forward_propagator(Base::_system), _controller_sampler()
  {
    prx::param_loader params;
    initialize(params);
  }

  rrt_memory_t(prx::param_loader params, prx::param_loader environment)
    : Base(params, environment), _forward_propagator(Base::_system), _controller_sampler(params["controller_sampler"])
  {
    initialize(params);
  }

  virtual void initialize(prx::param_loader params) override
  {
    Base::_nearest_neighbors->distance_function(
        [](const NodePtr& a, const NodePtr& b) { return System::distance(a->state(), b->state()); });
  }

  void sample(NodePtr node)
  {
    node->state() = Base::state_space()->sampler();
  }

  template <typename ControllerToSample, std::enable_if_t<std::is_same_v<ControllerToSample, Controller>, bool> = true>
  ControllerPtr sample()
  {
    return std::make_shared<Controller>(_controller_sampler());
  }

  TrajectoryPtr propagate(const State& x0, const ControllerPtr controller)
  {
    // Todo: make this create efficient by having a buffer
    TrajectoryPtr trajectory{ std::make_shared<Trajectory>() };
    // _forward_propagator(*trajectory, x0, *controller);
    return trajectory;
  }

  double cost(const TrajectoryPtr trajectory, const ControllerPtr controller)
  {
    return trajectory->duration();
  }
  // rrt_memory_t(const prx::param_loader params) :
  // {
  // }

  // ControlSpacePtr control_space()
  // {
  // }

private:
  DistanceFunction _distance_function;
  prx::sampler_t<Controller> _controller_sampler;
  ForwardPropagation _forward_propagator;
};

template <typename DynamicalSystem>
class rrt_functions_t : public planner_functions_t<rrt_memory_t<DynamicalSystem>>
{
public:
  using Base = planner_functions_t<rrt_memory_t<DynamicalSystem>>;
  using RRTMemory = rrt_memory_t<DynamicalSystem>;
  using Node = typename planner_specification_t<RRTMemory>::Node;
  using Edge = typename planner_specification_t<RRTMemory>::Edge;
  using Controller = typename planner_specification_t<RRTMemory>::Controller;
  using Trajectory = typename planner_specification_t<RRTMemory>::Trajectory;
  using NodeSelectionOut = prx::node_selection::interface_out_t<Node>;
  using NodeExpansionOut = prx::node_expansion::interface_out_t<Node, Controller, Trajectory>;
  using GraphUpdateOut = prx::graph_update::interface_out_t<Node, Edge>;
  using SolutionUpdateOut = prx::solution_update::interface_out_t;
  using TreeBNBInput = prx::solution_update::tree_bnb_input<Node>;
  using PlannerMemoryPtr = typename Base::PlannerMemoryPtr;

  rrt_functions_t()
    : Base()
    , _node_expansion_out(std::make_shared<NodeExpansionOut>())
    , _node_selection_out(std::make_shared<NodeSelectionOut>())
    , _graph_update_out(std::make_shared<GraphUpdateOut>())
    , _solution_update_out(std::make_shared<SolutionUpdateOut>())
  {
  }

  rrt_functions_t(prx::param_loader params) : Base(params)
  {
  }

  virtual void initialize(prx::param_loader params, PlannerMemoryPtr memory) override
  {
    memory->initialize(params);
    _node_selection_out = std::make_shared<NodeSelectionOut>();
    _node_selection_out->sample = std::make_shared<Node>();

    // queue doesn't have 'clear' <- easier to make a new one...
    _node_expansion_out = std::make_shared<NodeExpansionOut>();

    // _graph_update_out;
    _solution_update_out->goal_updated = false;
    _solution_update_out->goal_index = 0;
  }

  virtual void preprocess(PlannerMemoryPtr) override
  {
  }

  virtual bool condition_check(PlannerMemoryPtr memory) override
  {
    return memory->condition->check();
  }
  virtual void node_selection(PlannerMemoryPtr memory) override
  {
    prx::node_selection::voronoi_single_random(_node_selection_out, memory);
  }
  virtual void expand(PlannerMemoryPtr memory) override
  {
    prx::node_expansion::single_random_expansion(_node_expansion_out, _node_selection_out, memory);
  }
  virtual void node_validation(PlannerMemoryPtr memory) override
  {
    prx::node_validation::cost_to_come_check(_node_expansion_out, memory);
    prx::node_validation::trajectory_collision_check(_node_expansion_out, memory);
  }
  virtual void update_graph(PlannerMemoryPtr memory) override
  {
    prx::graph_update::add_edge_node_to_tree(_graph_update_out, _node_expansion_out, memory);
  }
  virtual void update_solution(PlannerMemoryPtr memory) override
  {
    prx::solution_update::update_solution_if_goal_found(_solution_update_out, _graph_update_out, memory);

    if (_solution_update_out->goal_updated)
    {
      _bnb_input->node_index = memory->root_node()->index();
      _bnb_input->delete_branch = false;
      _bnb_input->cost_bound = memory->statistics()->current_solution_cost;
      prx::solution_update::tree_branch_and_bound(_bnb_input, memory);
    }
  }
  virtual void update_stats(PlannerMemoryPtr memory) override
  {
    memory->statistics()->total_iterations++;
    memory->statistics()->total_nodes = memory->tree()->size();
    memory->statistics()->total_planning_time = memory->statistics()->_timer();
  }

  // virtual void answer_query(PlannerMemoryPtr) override
  // {
  // }

  virtual void postprocess(PlannerMemoryPtr memory) override
  {
    // prx::planner_postprocessing::recover_solution(std::shared_ptr<Output> output, memory);
  }

  virtual void reset(PlannerMemoryPtr) override
  {
  }

private:
  std::shared_ptr<NodeSelectionOut> _node_selection_out;
  std::shared_ptr<NodeExpansionOut> _node_expansion_out;
  std::shared_ptr<GraphUpdateOut> _graph_update_out;
  std::shared_ptr<SolutionUpdateOut> _solution_update_out;
  std::shared_ptr<TreeBNBInput> _bnb_input;
};

}  // namespace planners
}  // namespace prx
