#pragma once
#include <fstream>
#include <iostream>
#include <ostream>
#include <string>

#include <gtsam/nonlinear/GraphvizFormatting.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "prx/factor_graphs/utilities/prx_symbols.hpp"

namespace prx
{
namespace fg
{
class formatter_t : public gtsam::GraphvizFormatting
{
  using key_t = gtsam::Key;
  using value_t = std::function<Eigen::Vector3d(Eigen::VectorXd)>;

public:
  formatter_t() : GraphvizFormatting()
  {
    figureWidthInches = 50;
    figureHeightInches = 50;
  }

  void save_as_graphviz_file(const std::string& filename, const gtsam::NonlinearFactorGraph& graph,
                             const gtsam::Values& values,
                             const gtsam::KeyFormatter& keyFormatter = prx::key_formatter) const;

private:
  std::unordered_map<key_t, value_t> positions_memory;
};
}  // namespace fg
}  // namespace prx
