#pragma once

#include "prx/utilities/defs.hpp"
#include "prx/planning/planners/rrt.hpp"
#include "prx/utilities/general/timer.hpp"
#include "prx/planning/planners/planner.hpp"
#include "prx/utilities/data_structures/gnn.hpp"
#include "prx/utilities/data_structures/tree.hpp"
#include "prx/planning/planner_functions/planner_functions.hpp"

#include <stack>

namespace prx
{

class aorrt_node_t : public rrt_node_t
{
public:
  aorrt_node_t() : rrt_node_t()
  {
  }
  virtual ~aorrt_node_t()
  {
  }
  virtual void copy(const aorrt_node_t& other)
  {
    this->rrt_node_t::copy(other);
  }

  template <typename NodePtr, typename EdgePtr>
  void update(NodePtr parent_node, EdgePtr parent_edge)
  {
    rrt_node_t::update(parent_node, parent_edge);
  }
};

class aorrt_edge_t : public rrt_edge_t
{
public:
  aorrt_edge_t() : rrt_edge_t()
  {
  }
  virtual ~aorrt_edge_t()
  {
  }
  virtual void copy(const aorrt_edge_t& other)
  {
    this->rrt_edge_t::copy(other);
  }

  friend std::ostream& operator<<(std::ostream& os, const std::shared_ptr<aorrt_edge_t>& obj)
  {
    os << *obj;
    return os;
  }
  friend std::ostream& operator<<(std::ostream& os, aorrt_edge_t& obj)
  {
    os << static_cast<rrt_edge_t>(obj);
    return os;
  }
};

class aorrt_specification_t : public rrt_specification_t
{
public:
  aorrt_specification_t(std::shared_ptr<system_group_t> sg, std::shared_ptr<collision_group_t> cg)
    : rrt_specification_t(sg, cg)
    , cost_multiplier(1.0)
    , c_max(std::numeric_limits<double>::infinity())
    , w_x(1.0)
    , w_c(1.0)
  {
    // state_space = sg->get_state_space();

    //// Create the Y space = X U Cost
  }
  virtual ~aorrt_specification_t()
  {
  }

  static prx::param_loader init()
  {
    prx::param_loader params{ rrt_specification_t::init() };

    params["cost_multiplier"].set(decltype(cost_multiplier){});
    params["w_c"].set(decltype(w_c){});
    params["w_x"].set(decltype(w_x){});
    return params;
  }

  virtual void init(const prx::param_loader& params) override
  {
    rrt_specification_t::init(params);
    cost_multiplier = params.exists("cost_multiplier") ? params["cost_multiplier"].as<double>() : cost_multiplier;
    w_c = params.exists("w_c") ? params["w_c"].as<double>() : w_c;
    w_x = params.exists("w_x") ? params["w_x"].as<double>() : w_x;
  }

  friend std::ostream& operator<<(std::ostream& os, const aorrt_specification_t& obj)
  {
    // rrt_specification_t::operator<<(os, obj);
    os << static_cast<rrt_specification_t>(obj);
    os << "cost_multiplier: " << obj.cost_multiplier << "\n";
    os << "w_c: " << obj.w_c << "\n";
    os << "w_x: " << obj.w_x << "\n";

    return os;
  }

  // space_t* state_space;

  double c_max;  // Cost space will be sampled in [0, c_max * cost_multiplier]
  double w_c;
  double w_x;
  double cost_multiplier;
};

class aorrt_query_t : public rrt_query_t
{
public:
  aorrt_query_t(space_t* state_space, space_t* control_space) : rrt_query_t(state_space, control_space)
  {
    clear_outputs();
    goal_check = [&](const space_point_t& x) { return space_t::euclidean_2d(x, goal_state) < goal_region_radius; };
  }
  virtual ~aorrt_query_t()
  {
  }
};

class aorrt_t : public rrt_t
{
public:
  using Node = aorrt_node_t;
  using Edge = aorrt_edge_t;
  using EdgePtr = std::shared_ptr<Edge>;
  using NodePtr = std::shared_ptr<Node>;

  aorrt_t(const std::string& new_name);
  virtual ~aorrt_t();
  virtual std::vector<double> get_statistics() override;

  double get_sln_cost()
  {
    return _stats.current_solution_cost;
  };

  void print_statics();

  virtual std::shared_ptr<prx::tree_t> tree_of_solutions() override
  {
    // const double radius{ aorrt_query->goal_region_radius };
    // _cost_state_space->copy(_cost_aux_pt, { Y_min_cost });
    // Y_state_space->point_union(aorrt_query->goal_state, _cost_aux_pt, Y_aux_pt);
    const std::size_t total_solutions = aorrt_query->total_solutions;

    std::vector<prx::proximity_node_t*> goal_nodes{};

    using PairNodeCost = std::pair<prx::proximity_node_t*, double>;
    auto node_comp = [](const PairNodeCost& p1, const PairNodeCost& p2) { return p1.second < p2.second; };

    std::priority_queue<PairNodeCost, std::vector<PairNodeCost>, decltype(node_comp)> pq_nodes{ node_comp };

    auto pair_iters_vertices = _tree.vertices();
    for (auto iter = pair_iters_vertices.first; iter != pair_iters_vertices.second; iter++)
    {
      std::shared_ptr<tree_node_t> tree_node{ *iter };
      std::shared_ptr<Node> curr_node{ std::dynamic_pointer_cast<Node>(tree_node) };
      Y_state_space->split_point(curr_node->point, X_aux_pt, _cost_aux_pt);
      if (aorrt_query->goal_check(X_aux_pt))
      {
        const double dist{ distance_function(aorrt_query->goal_state, X_aux_pt) };
        pq_nodes.push(std::make_pair(tree_node.get(), dist));
        // PRX_DBG_VARS(X_aux_pt);
        // goal_nodes.push_back(tree_node.get());
      }
    }

    // PRX_DBG_VARS(total_solutions, pq_nodes.size());
    const std::size_t solutions{ std::min(total_solutions, pq_nodes.size()) };
    for (int i = 0; i < solutions; ++i)
    {
      goal_nodes.push_back(pq_nodes.top().first);
      pq_nodes.pop();
    }
    // PRX_DBG_VARS(goal_nodes.size());

    // const std::vector<prx::proximity_node_t*> goal_nodes{ metric->multi_query(Y_aux_pt, total_solutions) };
    return _tree_of_solutions<Node, Edge>(goal_nodes);
  }

  EdgePtr split_edge(EdgePtr e0, const double time_of_split)
  {
    // We get the edge0 and know the nodes: N0 -- E0 -- N1
    // After split, we have N0 -- E0 -- N2 -- E1 -- N1

    EdgePtr e1{ _tree.split_edge<Node, Edge>(e0->get_index()) };

    NodePtr n0{ _tree.get_vertex_as<Node>(e0->get_source()) };
    NodePtr n1{ _tree.get_vertex_as<Node>(e1->get_target()) };
    NodePtr n2{ _tree.get_vertex_as<Node>(e1->get_source()) };

    prx_assert(e0->get_target() == e1->get_source(), "Edges are not connected through a common node");
    // PRX_DBG_VARS(n0->get_index(), n1->get_index(), n2->get_index());
    // Split the plan and traj: Old = [ OldUpdated | New ]. AKA the new one correspond to the last part
    prx::plan_t e1_plan{ e0->plan->split(time_of_split) };
    prx::trajectory_t e1_traj_split{ e0->traj->split(time_of_split) };
    prx::trajectory_t e1_traj{ X_state_space };
    // PRX_DBG_VARS(e0->traj->size(), e1_traj_split.size());
    // prx_assert(e0->traj->size() > 0, "Traj of zero size, edge:" << e0->get_index());

    // e0->traj->push_back(e1_traj_split.front());
    e1_traj.push_back(e0->traj->back());
    e1_traj += e1_traj_split;
    const double e1_cost{ cost_function(e1_traj, e1_plan) };
    e1->update(e1_plan, e1_traj, e1_cost);

    // Update cost of e0 given the split
    e0->edge_cost = cost_function(*(e0->traj), *(e0->plan));

    Y_state_space->split_point(n0->point, X_aux_pt, _cost_aux_pt);
    _c_new = _cost_aux_pt->at(0) + e0->edge_cost;
    _cost_state_space->copy(_cost_aux_pt, { _c_new });
    Y_state_space->point_union(e0->traj->back(), _cost_aux_pt, Y_aux_pt);

    n2->point = Y_state_space->clone_point(Y_aux_pt);
    n2->update(n0, e0);
    return e1;
  }

protected:
  distance_function_t Y_distance_function = [&](const space_point_t& s1, const space_point_t& s2) {
    double cost{ 0.0 };

    if (Y_state_space->is_point_in_space(s1) && Y_state_space->is_point_in_space(s2))
    {
      Y_state_space->split_point(s1, _X_aux1, _cost_aux1);
      Y_state_space->split_point(s2, _X_aux2, _cost_aux2);
      cost += aorrt_spec->w_x * std::pow(Y_distance_function(_X_aux1, _X_aux2), 2);
      cost += aorrt_spec->w_c * std::pow(Y_distance_function(_cost_aux1, _cost_aux2), 2);
    }
    else if (X_state_space->is_point_in_space(s1) && X_state_space->is_point_in_space(s2))
    {
      cost += distance_function(s1, s2);
    }
    else if (_cost_state_space->is_point_in_space(s1) && _cost_state_space->is_point_in_space(s2))
    {
      cost += space_t::l2_norm(s1, s2);
    }
    else
    {
      printf("Points not in the same space!\n");
      exit(1);
    }

    return std::sqrt(cost);
  };

  virtual void update_goal(node_index_t node_index) override;

  virtual void _link_and_setup_spec(planner_specification_t* spec) override;
  virtual bool _preprocess() override;
  virtual bool _link_and_setup_query(planner_query_t* query) override;
  virtual void _resolve_query(condition_check_t* condition) override;
  virtual void _fulfill_query() override;
  virtual void _reset() override;

  virtual void bnb(node_index_t v, double cost_bound, bool delete_flag = false) override;

  aorrt_query_t* aorrt_query;
  aorrt_specification_t* aorrt_spec;

  heuristic_function_t heuristic;

  space_t* Y_state_space;
  space_t* X_state_space;
  // space_t* control_space;

  space_point_t X_sample_point;
  space_point_t Y_sample_point;

  space_point_t X_aux_pt;
  space_point_t Y_aux_pt;
  space_point_t _X_aux1, _X_aux2;

  // double Y_min_cost;
  double Y_min_g;
  space_point_t Y_min;

  std::vector<space_point_t> trajectory_costs;

  bool use_heuristic;

  int print_statics_count;

  // Cost space
  std::vector<double*> _cost_memory;
  space_point_t _cost_sample_point;
  space_t* _cost_state_space;
  space_point_t _cost_aux_pt;
  space_point_t _cost_aux1, _cost_aux2;

  double _c_new;
  double _w_x_bk;
  double _w_c_bk;
  double _c_max;
};
}  // namespace prx
