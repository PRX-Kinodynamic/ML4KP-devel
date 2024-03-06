#define BOOST_AUTO_TEST_MAIN world_model_test
#include <string>
// TODO: Change to <boost/test/unit_test.hpp>
#include <boost/test/unit_test.hpp>

#include "prx/planning/world_model.hpp"
#include "prx/simulation/plants/rally_car.hpp"
#include "prx/simulation/plants/treaded_vehicle.hpp"
#include "prx/simulation/plants/two_dimensional_point.hpp"
#include "prx/simulation/plants/three_dimensional_point.hpp"
#include "prx/simulation/plants/two_link_acrobot.hpp"
#include "prx/simulation/plants/fixed_wing.hpp"
#include "prx/simulation/plants/koules.hpp"
#include "prx/simulation/plants/racecar_mini.hpp"
#include "prx/simulation/plants/treaded_vehicle_first_order.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/utilities/geometry/basic_geoms/box.hpp"

BOOST_AUTO_TEST_CASE(empty_world_model_test)
{
  printf("Creating world model\n");
  prx::world_model_t world_model({}, {});

  printf("Creating context\n");
  world_model.create_context("test_context", {}, {});

  printf("Getting context\n");
  prx::world_model_context context = world_model.get_context("test_context");

  BOOST_CHECK(context.first != nullptr);
  BOOST_CHECK(context.second != nullptr);
  BOOST_CHECK(world_model.get_all_context_names()[0] == "test_context");
}

BOOST_AUTO_TEST_CASE(world_model_system_empty_environment_test)
{
  std::vector<std::string> system_names = { "2D_Point" };

  std::vector<prx::system_ptr_t> plants;

  prx::system_ptr_t sys_ptr = prx::system_factory_t::create_system("2D_Point", "2D_Point");
  plants.push_back(sys_ptr);

  printf("Creating world model\n");
  prx::world_model_t world_model({ plants }, {});

  printf("Creating context\n");
  world_model.create_context("test_context", { system_names }, {});

  printf("Getting context\n");
  prx::world_model_context context = world_model.get_context("test_context");

  BOOST_CHECK(context.first != nullptr);
  BOOST_CHECK(context.second != nullptr);
  BOOST_CHECK(world_model.get_all_context_names()[0] == "test_context");
}

BOOST_AUTO_TEST_CASE(world_model_3_systems_test)
{
  std::vector<std::string> system_names = { "2D_Point", "rally_car", "treaded_vehicle" };

  prx::transform_t obstacle_pose;
  obstacle_pose.setIdentity();
  auto box_obstacle = prx::create_obstacle(new prx::box_t("box", 2, 3, 4, obstacle_pose));

  // std::vector<std::shared_ptr<prx::movable_object_t>> obstacle_list = obstacles.second;
  // std::vector<std::string> obstacle_names = obstacles.first;

  std::vector<prx::system_ptr_t> plants;

  for (auto name : system_names)
  {
    auto sys_ptr = prx::system_factory_t::create_system(name, name);
    plants.push_back(sys_ptr);
  }
  // plants.push_back(prx::system_factory_t::create_system(system_names[0], system_names[0] + "_path"));
  // auto plant = prx::system_factory_t::create_system(system_names[0], system_names[0]);

  printf("Creating world model\n");
  prx::world_model_t world_model({ plants }, { box_obstacle });

  printf("Creating context\n");
  world_model.create_context("test_context", { system_names }, { "box" });

  printf("Getting context\n");
  auto context = world_model.get_context("test_context");

  BOOST_CHECK(context.first != nullptr);
  BOOST_CHECK(context.second != nullptr);
  BOOST_CHECK(world_model.get_all_context_names()[0] == "test_context");
}

BOOST_AUTO_TEST_CASE(world_model_adding_obstacles_dynamically)
{
  const std::string context_name{ "test_context" };
  const std::vector<std::string> system_names{ { "2D_Point" } };
  std::vector<prx::system_ptr_t> plants{ { prx::system_factory_t::create_system(system_names[0], system_names[0]) } };

  const Eigen::Vector2d position{ 0, 0 };
  prx::transform_t pose{ prx::transform_t::Identity() };
  pose.translation().head(3) = Eigen::Vector3d(1, 1, 0);

  plants[0]->get_state_space()->copy_from(position);
  const double obstacle_dim{ 2 };
  const std::string box_name{ "obstacle_0" };
  // World model is created without obstacles
  // std::shared_ptr<prx::box_t> box{ std::make_shared<prx::box_t>("obstacle_0", obstacle_dim, obstacle_dim,
  // obstacle_dim,
  // pose)
  // };

  prx::world_model_t world_model(plants, {});
  world_model.create_context(context_name, { system_names }, {});

  std::shared_ptr<prx::collision_group_t> collision_group{ world_model.collision_group(context_name) };

  BOOST_CHECK(collision_group != nullptr);

  PRX_DEBUG_ITERABLE("Distances:", collision_group->get_distances().distances);
  BOOST_CHECK(not collision_group->in_collision());  // No obstacles => no collisions
  // BOOST_CHECK(not collision_group->in_collision());  // No obstacles => no collisions

  // Check that this still works if adding box at the beginning
  world_model.emplace_obstacle<prx::box_t>(context_name, "obstacle_0", obstacle_dim, obstacle_dim, obstacle_dim, pose);

  BOOST_CHECK(collision_group->in_collision());  // The new obstacle is now in collision
}