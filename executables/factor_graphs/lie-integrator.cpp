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
// #include "prx/factor_graphs/defs.hpp"
#include "prx/factor_graphs/utilities/symbols_factory.hpp"

#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/linear/JacobianFactor.h>

int main(int argc, char* argv[])
{
  using prx::fg::screw_axis_t;
  using prx::fg::se3_t;
  using Integrator = prx::fg::lie_integrator_t<se3_t, screw_axis_t>;

  const Integrator integrator{};
  const double dt{ 0.1 };

  se3_t x{};
  screw_axis_t xdot{ screw_axis_t::from_twist(Eigen::Vector3d(0.0, 0.0, .10), Eigen::Vector3d(1.0, 0.10, 0)) };

  std::cout << "v:" << xdot.v().transpose() << "\n";
  std::cout << "w:" << xdot.omega().transpose() << "\n";
  std::cout << "se3: " << x << "\n";
  Eigen::AngleAxisd angle_axis{};
  for (double ti = 0.0; ti < 10; ti += dt)
  {
    x = integrator(x, xdot, dt);

    std::cout << x << "\n";
  }

  return 0;
}
