#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>

#include <boost/algorithm/string/join.hpp>
#include <boost/optional.hpp>

#include "prx/simulation/plants/plants.hpp"
#include "prx/simulation/playback/trajectory.hpp"
#include "prx/simulation/system_factory.hpp"
#include "prx/simulation/system_group.hpp"
#include "prx/planning/world_model.hpp"

#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/csv_reader.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/visualization/three_js_group.hpp"

#include "prx/factor_graphs/lie_groups/se3.hpp"
#include "prx/factor_graphs/lie_groups/screw_axis.hpp"
#include "prx/factor_graphs/lie_groups/lie_integrator.hpp"
#include "prx/factor_graphs/plants/pusher_slider.hpp"
#include "prx/factor_graphs/utilities/symbols_factory.hpp"
#include "prx/factor_graphs/utilities/default_parameters.hpp"

#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/basis/FitBasis.h>
#include <gtsam/basis/Chebyshev2.h>
#include <gtsam/basis/Chebyshev2.h>
#include <gtsam/basis/Basis.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/base/numericalDerivative.h>

#include <gtsam/basis/Chebyshev.h>

double f(const double x)
{
  // return sign - x / 2.0;
  return std::exp(x);
}

int main(int argc, char* argv[])
{
  auto model = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  // factor(key, measured, model, N, 0);
  int N = 6;
  gtsam::Key key(0);
  gtsam::NonlinearFactorGraph graph;
  double total{ 20.0 };
  for (double i = 0; i < total; ++i)
  {
    const double ti{ i / total };
    const double measured{ f(ti) };
    PRX_DBG_VARS(i, total, measured);

    graph.emplace_shared<gtsam::EvaluationFactor<gtsam::Chebyshev2>>(key, measured, model, N, ti, 0, 1);
    // graph.emplace_shared<gtsam::EvaluationFactor<gtsam::Chebyshev2Basis>>(key, measured, model, N, ti, 0, 1);
  }

  gtsam::Vector functionValues(N);
  functionValues.setZero();

  gtsam::Values initial;
  initial.insert<gtsam::Vector>(key, functionValues);

  gtsam::LevenbergMarquardtParams parameters;
  parameters.setVerbosityLM("SUMMARY");

  parameters.setMaxIterations(20);
  gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initial, parameters).optimize();
  functionValues = result.at<gtsam::Vector>(key);
  PRX_DBG_VARS(functionValues);

  for (double i = 0; i < 1; i += 0.01)
  {
    const gtsam::Chebyshev2::EvaluationFunctor func(N, i, 0, 1);
    // const gtsam::Chebyshev2Basis::EvaluationFunctor func(N, i, 0, 1);
    const double v{ func(functionValues) };
    PRX_DBG_VARS(v, f(i))
  }
}