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
#include "prx/factor_graphs/utilities/symbols_factory.hpp"
#include "prx/factor_graphs/plants/pusher_slider.hpp"

#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/basis/FitBasis.h>
#include <gtsam/basis/Chebyshev2.h>

using Sequence = std::map<double, double>;
using Sample = std::pair<double, double>;
using Weights = Eigen::Matrix<double, 1, -1>; /* 1xN vector */

double f(const double x)
{
  const double sign{ std::signbit(x) ? -1.0 : 1.0 };
  // return sign - x / 2.0;
  return std::sin(6.0 * x) + std::sin(60.0 * std::exp(x));
}

double p(const Eigen::VectorXd& ck, const double& x, const std::size_t& N)
{
  const Weights w{ gtsam::Chebyshev2::CalculateWeights(N, x) };
  return ck.dot(w);
}

int main(int argc, char* argv[])
{
  const std::size_t N{ 100 };
  Sequence sequence;
  for (int i = 0; i < 1000; ++i)
  {
    const double x{ prx::uniform_random<double>(-1.0, 1.0) };
    const double y{ f(x) + prx::gaussian_random(0.0, 0.1) };
    sequence[x] = y;
  }

  gtsam::noiseModel::Base::shared_ptr noise{ gtsam::noiseModel::Isotropic::Sigma(1, 0.01) };

  gtsam::FitBasis<gtsam::Chebyshev2> fit(sequence, noise, N);

  gtsam::Chebyshev2::Parameters params{ fit.parameters() };
  // Eigen::VectorXd Tn{ gtsam::Chebyshev2::Points(N) };

  // PRX_DBG_VARS(Tn);
  // PRX_DBG_VARS(params);

  for (double xi = -1.0; xi < 1.0; xi += 0.01)
  {
    std::cout << xi << " " << f(xi) << " " << p(params, xi, N) << "\n";
  }
  // PRX_DBG_VARS(pk);
}