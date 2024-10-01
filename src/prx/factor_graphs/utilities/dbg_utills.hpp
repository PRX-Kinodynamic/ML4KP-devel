#pragma once
#include <fstream>
#include <gtsam/nonlinear/Values.h>
#include "prx/factor_graphs/utilities/symbols_factory.hpp"

namespace prx
{
namespace fg
{
using SF = prx::fg::symbol_factory_t;

void indeterminant_linear_system_helper(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
                                        std::ostream& os = std::cout)
{
  boost::shared_ptr<gtsam::GaussianFactorGraph> fgl{ graph.linearize(values) };
  os << "Keys:\n";
  int idx{ 0 };
  for (auto k : fgl->keys())
  {
    os << "[" << idx << "]: " << SF::formatter(k) << "\n";
    idx++;
  }
  os << "Use Matlab's 'sparse()':\n";
  os << fgl->sparseJacobian_() << "\n";

  os << "Jacobian A|b':\n";
  os << "\tA:\n";
  auto Ab = fgl->jacobian();
  os << Ab.first << "\n";
  os << "\tb:\n";
  os << Ab.second << "\n";

  auto H = fgl->hessian();
  os << "Hessian A|b':\n";
  os << "\tA:\n";
  os << H.first << "\n";
  os << "\tb:\n";
  os << H.second << "\n";
  // std::pair<Matrix, Vector>
}

}  // namespace fg
}  // namespace prx
