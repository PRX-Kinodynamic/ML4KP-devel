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
    cost_to_go = 0;
  }
  virtual ~aorrt_node_t()
  {
  }

  double cost_to_go;
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
};

class cost_space_t : public space_t
{
  // friend class space_t;
public:
  cost_space_t() : space_t()
  {
    owned_values = true;
    addresses.clear();
    addresses.push_back(&_cost);

    dimension = addresses.size();

    topology.clear();
    lower_bounds.clear();
    upper_bounds.clear();

    topology.push_back(topology_t::EUCLIDEAN);
    lower_bounds.push_back(new double(0));
    upper_bounds.push_back(new double(0));

    space_name = "CostState";

    start_state = make_point();
    copy_point_from_vector(start_state, { 0 });
  }

  space_point_t get_start_state()
  {
    return start_state;
  }

  void set_cost(const space_point_t& pt, double cost)
  {
    pt->at(0) = cost;
  }

  virtual ~cost_space_t()
  {
  }

protected:
  double _cost;
  space_point_t start_state;
};

class aorrt_specification_t : public rrt_specification_t
{
public:
  aorrt_specification_t(std::shared_ptr<system_group_t> sg, std::shared_ptr<collision_group_t> cg)
    : rrt_specification_t(sg, cg), _debug(false)
  {
    Y_min_cost = std::numeric_limits<double>::infinity();

    X_state_space = sg->get_state_space();

    c_max = std::numeric_limits<double>::infinity();

    //// Create the cost space
    cost_state_space = new cost_space_t();

    //// Create the Y space = X U Cost
    Y_state_space = new space_t({ X_state_space, cost_state_space });

    X_aux1 = X_state_space->make_point();
    X_aux2 = X_state_space->make_point();
    cost_aux1 = cost_state_space->make_point();
    cost_aux2 = cost_state_space->make_point();
    w_x = 1;
    w_c = 1;

    Y_distance_function = [this](const space_point_t& s1, const space_point_t& s2) {
      double cost{ -1 };

      if (Y_state_space->is_point_in_space(s1) && Y_state_space->is_point_in_space(s2))
      {
        Y_state_space->split_point(s1, X_aux1, cost_aux1);
        Y_state_space->split_point(s2, X_aux2, cost_aux2);
        cost = w_x * std::pow(distance_function(X_aux1, X_aux2), 2);
        cost += w_c * std::pow(Y_distance_function(cost_aux1, cost_aux2), 2);
      }
      else if (X_state_space->is_point_in_space(s1) && X_state_space->is_point_in_space(s2))
      {
        prx_throw("X_state_space");
      }
      else if (cost_state_space->is_point_in_space(s1) && cost_state_space->is_point_in_space(s2))
      {
        cost = std::pow(s1->at(0) - s2->at(0), 2);
      }
      else
      {
        printf("Points not in the same space!\n");
        exit(1);
      }
      prx_assert(cost >= 0, "Negative cost found: " << cost);

      return std::sqrt(cost);
    };
  }
  virtual ~aorrt_specification_t()
  {
  }
  distance_function_t Y_distance_function;

  space_t* X_state_space;
  space_t* Y_state_space;

  space_point_t X_aux1, X_aux2;
  space_point_t cost_aux1, cost_aux2;

  cost_space_t* cost_state_space;

  double c_max;
  double w_c;
  double w_x;
  double Y_min_cost;
  bool _debug;
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
  aorrt_t(const std::string& new_name);
  virtual ~aorrt_t();
  virtual std::vector<double> get_statistics() override;

  double get_sln_cost()
  {
    return Y_min_cost;
  };

  void print_statics();

protected:
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

  graph_nearest_neighbors_t* metric_rrt;

  space_t* X_state_space;
  space_t* Y_state_space;

  // Cost variables
  cost_space_t* cost_state_space;

  space_point_t X_sample_point;
  space_point_t Y_sample_point;
  space_point_t cost_sample_point;

  space_point_t X_aux_pt;
  space_point_t Y_aux_pt;
  space_point_t cost_aux_pt;

  double Y_min_cost;
  double Y_min_g;
  space_point_t Y_min;

  double c_max;
  std::vector<space_point_t> trajectory_costs;

  bool use_heuristic;

  double c_new;
  double w_x_bk;
  double w_c_bk;
};
}  // namespace prx
