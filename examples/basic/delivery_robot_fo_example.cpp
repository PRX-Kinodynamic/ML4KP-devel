#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/param_loader.hpp"

#include "prx/planning/world_model.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/simulation/plants/delivery_robot_fo.hpp"

using namespace prx;

int main(int argc, char* argv[])
{
  simulation_step = 0.1;
  init_random(210896);

  //todo: Load params from yaml
  auto plant = prx::system_factory_t::create_system("FO_delivery_robot", "FO_delivery_robot");
  prx_assert(plant != nullptr, "Plant is nullptr!");

  world_model_t world_model({ plant }, {});
  world_model.create_context("context", { "FO_delivery_robot" }, {});
  
  auto context = world_model.get_context("context");
  auto system_group = context.first;
  prx::space_t* ss{ system_group->get_state_space() };
  prx::space_t* cs{ system_group->get_control_space() };

  auto start_state = ss->make_point();
  ss->copy(start_state, Eigen::Vector<double, 4>(0, 0, 0, 0));

  plan_t plan(cs);
  trajectory_t traj(ss);

  // Declare a plan that is always picking up a package
  plan.copy_onto_back(Eigen::Vector3d(0.7,0.7,1.0), 5);
  const double package_mass{0.2};

  std::function<void()> package_pickup = [&]()
  {
    const double x{ss->at(0)};
    const double y{ss->at(1)};

    // There is a package pickup between (1.0, 0.0) and (2.0, 0.0)
    // of mass 0.2kg
    if (x > 1.0 && x < 2.0 && cs->at(2) > 0)
    {
      // todo: Add checks
      const std::vector<double> clb = {-0.7+package_mass, -0.7+package_mass, 0};
      const std::vector<double> cub = {0.7-package_mass, 0.7-package_mass, 1};
      cs->set_bounds(clb, cub);
    }
  };

  world_model.world_change_callback = package_pickup;

  system_group->propagate(start_state, plan, traj);

  // todo: Add visualization
}