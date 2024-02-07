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

#include "prx/factor_graphs/factors/SE3.hpp"
#include "prx/factor_graphs/factors/screw_axis.hpp"
#include "prx/factor_graphs/factors/preintegration.hpp"
// #include "prx/factor_graphs/defs.hpp"
#include "prx/factor_graphs/utilities/symbols_factory.hpp"

#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/linear/JacobianFactor.h>

int main(int argc, char* argv[])
{
  using prx::fg::screw_axis_t;
  using prx::fg::SE3_t;

  const double dt{ 0.1 };
  prx::fg::preintegration_t<SE3_t, screw_axis_t> preint{ dt };

  SE3_t x{ SE3_t::Base::Identity() };
  screw_axis_t xdot{ screw_axis_t::from_twist(Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d(1.0, 0.1, 0)) };

  std::cout << "v:" << xdot.v().transpose() << "\n";
  std::cout << "w:" << xdot.omega().transpose() << "\n";
  std::cout << x.translation().transpose() << "\n";
  Eigen::AngleAxisd angle_axis{};
  for (double ti = 0.0; ti < 10; ti += dt)
  {
    x = preint.propagate(x, xdot);
    const double theta{ prx::quaternion_to_euler(Eigen::Quaterniond(x.rotation()))[2] };

    std::cout << x.translation().transpose() << " ";
    std::cout << theta << "\n";
  }

  return 0;
}
