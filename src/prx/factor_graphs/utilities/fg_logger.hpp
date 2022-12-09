#pragma once

#include <chrono>

#include "prx/utilities/general/logger.hpp"

#include "prx/factor_graphs/factors/factors.hpp"
#include "prx/factor_graphs/utilities/prx_symbols.hpp"

#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace prx
{
typedef std::function<bool(const gtsam::Factor* /* Factor */, double /* whitenedError */, size_t /* index */)>
    factor_filter_t;
// static const factor_filter_t factor_filter_default = [](const gtsam::Factor *, double, size_t) {return true;};

inline bool default_factor_filter(const gtsam::Factor*, double, size_t)
{
  return true;
}

std::string get_factor_name(const gtsam::NonlinearFactorGraph::sharedFactor& factor);
// {
// 	// if (std::dynamic_pointer_cast<goal_distance_factor_t>(factor))
// 	GET_FACTOR_NAME_MACRO(goal_distance_factor_t, factor, "goal_factor")
// 	GET_FACTOR_NAME_MACRO(propagation_factor_t, factor, "propagation_factor")
// 	GET_FACTOR_NAME_MACRO(space_limit_factor_t, factor, "space_limit_factor")
// 	GET_FACTOR_NAME_MACRO(gtsam::PriorFactor<Eigen::VectorXd>, factor, "prior_factor")

// 	return "no_factor_name";
// }

class fg_logger_t : public logger_t
{
public:
  fg_logger_t(const std::string& file_name, char separator = ' ', const std::string& _nullptr_val = "-")
    : logger_t(file_name, separator = ' ')
  {
    nullptr_val = _nullptr_val;
    // ofstream ofs_log(params_.logFile.c_str(), ios::trunc);
  }

  virtual ~fg_logger_t()
  {
  }

  void add_graph_errors(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
                        const std::string& extra_values = "",
                        const factor_filter_t& print_condition = default_factor_filter,
                        const gtsam::KeyFormatter& kf = prx::key_formatter);
  // static const factor_filter_t factor_filter_default = default_factor_filter;

private:
  // std::shared_ptr<gtsam::NonlinearFactorGraph> graph_ptr;
  std::string nullptr_val;
};
}  // namespace prx