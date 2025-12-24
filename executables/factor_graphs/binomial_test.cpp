#include "prx/factor_graphs/factors/obstacle_factor.hpp"
#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/aorrt.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/planners/planner.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/planning/planner_functions/tree_fix_time_discretization.hpp"

int main(int argc, char* argv[])
{
  prx::param_loader params{};
  params["N"].set(100);
  params.add_opts(argc, argv);

  std::random_device rd;
  std::mt19937 gen(rd());

  const double p{ params["p"].as<double>() };
  const double N{ params["N"].as<double>() };

  // std::ofstream ofs_map.open(prx::out_path + "binomial.txt", _mode);

  // for (int i = 1; i < N; ++i)
  // {
  //   std::bernoulli_distribution d(i, p);
  //   const double r{ d(gen) };
  // }

  return 0;
}