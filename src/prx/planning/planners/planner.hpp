#pragma once

// #include "general/param_loader.hpp"
#include <sstream>
// #include "general/statistics.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/condition_check.hpp"
// #include "prx/utilities/general/statistics.hpp"

#include "prx/simulation/playback/plan.hpp"
#include "prx/simulation/playback/trajectory.hpp"

namespace prx
{
/**
 * @brief <b>A class that specified the parameters of a planner.</b>
 * @authors Zakary Littlefield
 */

// TODO: Don't like this being here and not in planner_functions... ideas?
typedef std::function<bool(space_point_t&)> goal_check_t;

template <typename PlannerSpec, typename PlannerQuery, typename Planner>
param_loader init()
{
  param_loader params;
  // params["PlannerSpec"] = prx::param_loader();
  // params["PlannerQuery"] = prx::param_loader();
  // params["Planner"] = prx::param_loader();

  params["PlannerSpec"] = PlannerSpec::init();
  params["PlannerQuery"] = PlannerQuery::init();
  params["Planner"] = Planner::init();
  return params;
}

class planner_specification_t
{
public:
  planner_specification_t()
  {
  }
  virtual ~planner_specification_t()
  {
  }
  static prx::param_loader init()
  {
    return prx::param_loader();
  };
  virtual void init(const prx::param_loader& params) {};
};

class planner_query_t
{
public:
  planner_query_t(space_t* state_space, space_t* control_space)
    : solution_traj(state_space)
    , solution_plan(control_space)
    , start_state(state_space->make_point())
    , goal_state(state_space->make_point())
    , get_visualization(false)
  {
  }
  virtual ~planner_query_t()
  {
  }

  virtual void clear_outputs()
  {
    solution_traj.clear();
    solution_plan.clear();
    solution_cost = 0;
    tree_visualization.clear();
  }

  static prx::param_loader init()
  {
    prx::param_loader params;
    // param_loader params;
    params["start_state"] = space_snapshot_t::init();

    params["goal"] = prx::param_loader();
    params["goal/state"] = space_snapshot_t::init();
    params["visualize"].set(decltype(get_visualization){});
    return params;
  }

  virtual void init(const prx::param_loader& params)
  {
    if (start_state and params.exists("start_state"))
    {
      start_state->init(params["start_state"]);
    }
    if (goal_state and params.exists("goal"))
    {
      const prx::param_loader params_goal{ params["goal"] };
      if (params_goal.exists("state"))
      {
        goal_state->init(params_goal["state"]);
      }
    }
    get_visualization = params.exists("visualize") ? params["visualize"].as<bool>() : get_visualization;
  }

  friend std::ostream& operator<<(std::ostream& os, const planner_query_t& obj)
  {
    os << "start_state: " << obj.start_state << "\n";
    os << "goal_state: " << obj.goal_state << "\n";
    os << "get_visualization: " << obj.get_visualization << "\n";

    return os;
  }
  // inputs
  space_point_t start_state;
  space_point_t goal_state;
  goal_check_t goal_check;
  bool get_visualization;

  // outputs
  trajectory_t solution_traj;
  plan_t solution_plan;
  double solution_cost;
  std::vector<trajectory_t> tree_visualization;
};

/**
 *    +------------->Construction
 *	  |                   +
 *	  |                   |
 *	  |    +-----------+  |
 *	  |    |           v  v
 *	  |  +-^------Link and Setup Spec
 *	  |  | |              +   + ^
 *	  |  | |              |   +-+
 *	  |  | |              |
 *	  +  | +^---------+   v
 *	Reset<-----------+Preprocess
 *	  ^    |              +  + ^
 *	  |    |              |  +-+
 *	  |    |              |
 *	  |    +^----+        v
 *	  +^--------+Link and Setup Query
 *	  |    |              +   + ^ ^ ^
 *	  |    |              |   +-+ | |
 *	  |    |              |       | |
 *	  |    +^--------+    ^       | |
 *	  +^------------+Resolve Query+ |
 *	  |    |              |   + ^   |
 *	  |    |              |   +-+   |
 *	  |    |              |         |
 *	  |    +---------+    v         |
 *	  +-------------+Finalize Query++
 *
 */
class planner_t : public std::enable_shared_from_this<planner_t>
{
public:
  struct statistics_t
  {
    statistics_t() : statistics_t(-1., 0, 0, -1., -1., 0) {};

    statistics_t(const double planned_duration_, const std::size_t iteration_count_, const std::size_t total_nodes_,
                 const double cost_current_solution_, const double time_current_solution_,
                 const std::size_t iters_current_solution_)
      : planned_duration(planned_duration_)
      , iteration_count(iteration_count_)
      , total_nodes(total_nodes_)
      , cost_current_solution(cost_current_solution_)
      , time_current_solution(time_current_solution_)
      , iters_current_solution(iters_current_solution_)
    {
    }
    virtual ~statistics_t() {};

    const double planned_duration;      // timer.measure()};
    const std::size_t iteration_count;  // static_cast<double>(iteration_count);
    const std::size_t total_nodes;      // static_cast<double>(metric->get_nr_nodes());
    const double cost_current_solution;
    const double time_current_solution;
    const std::size_t iters_current_solution;

    static std::string header()
    {
      std::stringstream strstr;
      strstr << "planned_duration ";
      strstr << "iteration_count ";
      strstr << "total_nodes ";
      strstr << "cost_current_solution ";
      strstr << "time_current_solution ";
      strstr << "iters_current_solution ";
      return strstr.str();
    }

    friend std::ostream& operator<<(std::ostream& os, const statistics_t& obj)
    {
      os << obj.planned_duration << " ";
      os << obj.iteration_count << " ";
      os << obj.total_nodes << " ";
      os << obj.cost_current_solution << " ";
      os << obj.time_current_solution << " ";
      os << obj.iters_current_solution << " ";
      return os;
    }
  };

  using StatisticsPtr = std::shared_ptr<planner_t::statistics_t>;
  planner_t(const std::string& new_name);
  virtual ~planner_t();

  static prx::param_loader init()
  {
    param_loader params;
    params["name"].set(decltype(_planner_name){});
    return params;
  }

  /**
   *	Previous Function: Any function (Reset will be called)
   */
  void link_and_setup_spec(planner_specification_t* spec);
  // void link_and_setup_spec_shared(std::shared_ptr<planner_specification_t> spec);

  /**
   *	Previous Function: Link spec or preprocess
   */
  bool preprocess();

  /**
   *	Previous Function: Preprocess, Query functions
   */
  bool link_and_setup_query(planner_query_t* query);
  // bool link_and_setup_query_shared(std::shared_ptr<planner_query_t> query);

  /**
   *	Previous Function: Link and resolve query
   */
  void resolve_query(condition_check_t* condition);

  /**
   *	Previous Function: Resolve Query
   */
  void fulfill_query();

  /**
   *	Previous Function: Any function.
   */
  void reset();

  virtual std::vector<std::string> get_statistics_header()
  {
    return {};
  }

  // virtual statistics_t statistics()
  virtual StatisticsPtr statistics()
  {
    return nullptr;
  }

  virtual std::vector<double> get_statistics()
  {
    return {};
  }

protected:
  virtual void _link_and_setup_spec(planner_specification_t* spec) = 0;
  virtual bool _preprocess() = 0;
  virtual bool _link_and_setup_query(planner_query_t* query) = 0;
  virtual void _resolve_query(condition_check_t* condition) = 0;
  virtual void _fulfill_query() = 0;
  virtual void _reset() = 0;

  // virtual void _link_and_setup_spec_shared(std::shared_ptr<planner_specification_t> spec)=0;
  // virtual bool _link_and_setup_query_shared(std::shared_ptr<planner_query_t> query)=0;
  std::string _planner_name;

private:
  std::string get_current_stage_name();

  enum class planner_stage_t
  {
    CONSTRUCTION,
    LINK_SPECIFICATION,
    PREPROCESS,
    LINK_QUERY,
    RESOLVE_QUERY,
    FULFILL_QUERY
  } planner_state;
};
}  // namespace prx
