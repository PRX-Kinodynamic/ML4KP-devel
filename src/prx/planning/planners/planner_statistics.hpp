#include <memory>
#include <queue>
#include "general/debug_utils.hpp"
#include "general/prx_assert.hpp"
#include "prx/utilities/general/timer.hpp"
namespace prx
{
namespace planner_statistics
{
struct tree_statistics_t
{
  tree_statistics_t()
  {
    reset();
  }

  tree_statistics_t(const tree_statistics_t& other) = default;

  virtual ~tree_statistics_t() {};

  static std::string header()
  {
    std::stringstream strstr;
    strstr << "total_planning_time ";
    strstr << "total_iterations ";
    strstr << "total_nodes ";
    strstr << "solution_found ";
    strstr << "current_solution_cost ";
    strstr << "current_solution_time ";
    strstr << "current_solution_iterations ";
    strstr << "first_solution_cost ";
    strstr << "first_solution_time ";
    strstr << "first_solution_iterations ";
    return strstr.str();
  }

  friend std::ostream& operator<<(std::ostream& os, const tree_statistics_t& obj)
  {
    os << obj.total_planning_time << " ";
    os << obj.total_iterations << " ";
    os << obj.total_nodes << " ";
    os << (obj.solution_found ? "true" : "false") << " ";
    os << obj.current_solution_cost << " ";
    os << obj.current_solution_time << " ";
    os << obj.current_solution_iterations << " ";
    os << obj.first_solution_cost << " ";
    os << obj.first_solution_time << " ";
    os << obj.first_solution_iterations << " ";
    return os;
  }

  virtual void update_solution(const double current_solution_cost_)
  {
    current_solution_cost = current_solution_cost_;
    current_solution_time = _timer();
    current_solution_iterations = total_iterations;
    if (not solution_found)
    {
      first_solution_cost = current_solution_cost_;
      first_solution_time = current_solution_time;
      first_solution_iterations = total_iterations;
      solution_found = true;
    }
  }

  virtual void reset()
  {
    _timer.reset();
    solution_found = false;
    total_planning_time = std::numeric_limits<double>::infinity();
    current_solution_cost = std::numeric_limits<double>::infinity();
    current_solution_time = std::numeric_limits<double>::infinity();
    first_solution_cost = std::numeric_limits<double>::infinity();
    first_solution_time = std::numeric_limits<double>::infinity();
    current_solution_iterations = 0;
    first_solution_iterations = 0;
  }

  prx::timer_t _timer;
  // Planner stats
  double total_planning_time;
  std::size_t total_iterations;
  std::size_t total_nodes;

  // Solution-specific stats
  bool solution_found;
  double current_solution_cost;
  double current_solution_time;
  std::size_t current_solution_iterations;
  double first_solution_cost;
  double first_solution_time;
  std::size_t first_solution_iterations;
};

}  // namespace planner_statistics
}  // namespace prx