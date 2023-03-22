#define BOOST_AUTO_TEST_MAIN time_map_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/simulation/multivalued_map/systems.hpp"
#include "prx/simulation/plants/plants.hpp"
using prx::simulation::time_map_t;

BOOST_AUTO_TEST_CASE(time_map_pendulum_lqr_is_built_correctly)
{
  const std::string plant_name{ "pendulum" };
  prx::system_ptr_t plant = prx::system_factory_t::create_system(plant_name, plant_name);
  BOOST_REQUIRE(plant != nullptr);
  prx::world_model_t* world_model = new prx::world_model_t({ plant }, {});
  world_model->create_context("context", { plant_name }, {});
  auto context = world_model->get_context("context");
  std::shared_ptr<prx::system_group_t> system_group = context.first;

  time_map_t time_map("pendulum_lqr", plant, system_group);

  BOOST_CHECK_MESSAGE(time_map.x_goal != nullptr, "x_goal should not be a nullptr");
  BOOST_CHECK_MESSAGE(time_map.u_goal != nullptr, "u_goal should not be a nullptr");
}

BOOST_AUTO_TEST_CASE(time_map_pendulum_lqr_end_state_stays_in_zero_equilibrium)
{
  const std::string plant_name{ "pendulum" };
  prx::system_ptr_t plant = prx::system_factory_t::create_system(plant_name, plant_name);
  BOOST_REQUIRE(plant != nullptr);
  prx::world_model_t* world_model = new prx::world_model_t({ plant }, {});
  world_model->create_context("context", { plant_name }, {});
  auto context = world_model->get_context("context");
  std::shared_ptr<prx::system_group_t> system_group = context.first;

  const Eigen::Vector2d start_state{ 0.1, 0.1 };
  Eigen::Vector2d end_state;
  time_map_t time_map("pendulum_lqr", plant, system_group);
  time_map.set_duration(5);
  time_map(start_state, end_state);
  BOOST_CHECK_SMALL(end_state.norm(), 1e-3);
}

BOOST_AUTO_TEST_CASE(time_map_pendulum_lqr_traj_stays_in_zero_equilibrium)
{
  const std::string plant_name{ "pendulum" };
  prx::system_ptr_t plant = prx::system_factory_t::create_system(plant_name, plant_name);
  BOOST_REQUIRE(plant != nullptr);
  prx::world_model_t* world_model = new prx::world_model_t({ plant }, {});
  world_model->create_context("context", { plant_name }, {});
  auto context = world_model->get_context("context");
  std::shared_ptr<prx::system_group_t> system_group = context.first;

  const Eigen::Vector2d start_state{ 0.1, 0.1 };
  prx::trajectory_t result_traj(system_group->get_state_space());
  time_map_t time_map("pendulum_lqr", plant, system_group);
  time_map.set_duration(5);
  time_map(start_state, result_traj);
  BOOST_CHECK_SMALL(result_traj.back()->vector().norm(), 1e-3);
}