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
  world_model.emplace_obstacle<prx::box_t>(context_name, box_name, obstacle_dim, obstacle_dim, obstacle_dim, pose);

  BOOST_CHECK(collision_group->in_collision());  // The new obstacle is now in collision

  std::shared_ptr<prx::movable_object_t> box_movable{ world_model.obstacle(box_name) };
  BOOST_CHECK(box_movable != nullptr);  // The new obstacle is now in collision
  std::shared_ptr<prx::transform_t> box_tf{ box_movable->transform_ptr("body") };
  BOOST_CHECK(box_tf != nullptr);                          // The new obstacle is now in collision
  box_tf->translation() = Eigen::Vector3d(100, 100, 100);  // Move it out so there is no collision
  BOOST_CHECK(not collision_group->in_collision());
}

class test_plant_t : public prx::plant_t
{
public:
  test_plant_t() : prx::plant_t("test_plant")
  {
    x = y = z = 0;
    qx = qy = qz = 0;
    qw = 1;

    state_memory = { &x, &y, &z, &qx, &qy, &qz, &qw };
    state_space = new prx::space_t("EEEQQQQ", state_memory, "plant_state");
    control_memory = { &idle };
    input_control_space = new prx::space_t("I", control_memory, "plant_control");

    geometries["body"] = std::make_shared<prx::geometry_t>(prx::geometry_type_t::OBJ);
    geometries["body"]->initialize_obj_geometry(prx::obj_models_path + "test_peg.obj");
    geometries["body"]->generate_collision_geometry();
    configurations["body"] = std::make_shared<prx::transform_t>();
    configurations["body"]->setIdentity();
  }

  virtual void update_configuration()
  {
    configurations["body"]->setIdentity();
    configurations["body"]->translation() = (prx::vector_t(x, y, z));
    configurations["body"]->linear() = prx::quaternion_t(qw, qx, qy, qz).toRotationMatrix();
  };
  virtual void compute_derivative(){};

  double x, y, z, qx, qy, qz, qw, idle;
};

BOOST_AUTO_TEST_CASE(world_model_obstacles_from_obj)
{
  const std::vector<std::string> system_names{ { "peg" } };
  std::shared_ptr<test_plant_t> test_plant_ptr = std::make_shared<test_plant_t>();
  std::shared_ptr<prx::plant_t> plant_ptr = std::static_pointer_cast<prx::plant_t>(test_plant_ptr);
  std::shared_ptr<prx::system_t> system_ptr = std::static_pointer_cast<prx::system_t>(plant_ptr);
  BOOST_CHECK(system_ptr != nullptr);

  prx::obstacle_loader_t obstacle_loader = prx::obstacle_loader_t("environments/obstacle_obj.yaml");
  std::vector<std::string> obstacles_names = obstacle_loader.get_names();
  std::vector<std::shared_ptr<prx::movable_object_t>> obstacles = obstacle_loader.get_obstacles();

  std::vector<prx::system_ptr_t> plants;
  plants.push_back(system_ptr);
  std::shared_ptr<prx::collision_group_t> collision_group = std::make_shared<prx::collision_group_t>(plants, obstacles);

  BOOST_CHECK(collision_group != nullptr);

  // Peg is outside the hole
  std::vector<double> state_vector = { 0.0, 0.0, 0.07, 0.0, 0.0, 0.0, 1.0 };
  plant_ptr->get_state_space()->copy_from(state_vector);

  BOOST_CHECK(not collision_group->in_collision());

  // Peg is inside the hole
  state_vector = { 0.0, 0.0, 0.025, 0.0, 0.0, 0.0, 1.0 };
  plant_ptr->get_state_space()->copy_from(state_vector);

  BOOST_CHECK(collision_group->in_collision());
}