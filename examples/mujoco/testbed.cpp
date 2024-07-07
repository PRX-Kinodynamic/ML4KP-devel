// #ifndef TORCH_NOT_BUILT
#include "prx/utilities/defs.hpp"
#include "prx/mujoco/mj_simulator.hpp"
#include "prx/planning/planners/dirt.hpp"

#include <boost/filesystem.hpp>
using namespace boost::filesystem;

using namespace prx;

int main(int argc, char* argv[])
{
  param_loader params;

  params = param_loader("examples/JIST/open.yaml");
  init_random(params["random_seed"].as<int>());

  bool visualize = params["visualize"].as<bool>();
  bool visualize_tree = params["visualize_tree"].as<bool>();

  std::vector<std::pair<std::string, std::string>> ignored_pairs = params["ignored_pairs"].as<std::vector<std::pair<std::string, std::string>>>();

  std::shared_ptr<prx::mujoco_simulator_t> sim =
      std::make_shared<prx::mujoco_simulator_t>(params["xml_path"].as<std::string>(), visualize);
  sim->init_simulator();

  sim->add_pair(ignored_pairs);

  auto context = sim->get_context("mujoco");
  auto ss = context.first->get_state_space();
  auto cs = context.first->get_control_space();


  for (int i = 0; i < 5; i++)
  {
    sim->step_simulation();
  }
  std::cout << ss->print_memory() << std::endl;

  dirt_t dirt(params["planner_name"].as<>());
  dirt_specification_t dirt_spec(context.first, context.second);

  float min_control_scaling = 0.1;
  float max_control_scaling = 1.0;

    plan_t plan(cs);
    plan.append_onto_back(1000);
    plan.back().control->at(0) = 1;
    space_point_t start = ss -> make_point();
    ss->copy_to_point(start);

    trajectory_t traj(ss);
    dirt_spec.propagate(start, plan, traj);
    std::cout << ss->print_point(traj.back()) << std::endl;

  std::cout << "End of program!" << std::endl;
}

/*
#else
int main()
{
  std::cout << "Torch not built!" << std::endl;
}
#endif
*/