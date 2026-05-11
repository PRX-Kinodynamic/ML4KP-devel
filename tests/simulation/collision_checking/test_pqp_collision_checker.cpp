#include <memory>
#include "geometry/basic_geoms/box.hpp"
#define BOOST_AUTO_TEST_MAIN pqp_collision_checker
#include <string>
#include <boost/test/unit_test.hpp>

// #include "general/constants.hpp"
#include "prx/utilities/geometry/basic_geoms/sphere.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/simulation/collision_checking/pqp_collision_checker.hpp"

namespace mock
{
using State = Eigen::Vector3d;
struct plant_t
{
  std::vector<std::shared_ptr<prx::geometry_t>> geometries()
  {
    std::vector<std::shared_ptr<prx::geometry_t>> geometries;
    geometries.push_back(std::make_shared<prx::geometry_t>(prx::geometry_type_t::SPHERE));
    geometries.back()->initialize_geometry({ 0.5 });
    geometries.back()->generate_collision_geometry();
    geometries.back()->set_visualization_color("0x00ff00");
    return geometries;
  }
  std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> configuration(const State& state)
  {
    const Eigen::Matrix3d R{ prx::axis_to_rotation_matrix({ state[2] }, 'Z') };
    const Eigen::Vector3d t(state[0], state[1], 0.0);
    return { { R, t } };
  }
};
}  // namespace mock

BOOST_AUTO_TEST_CASE(test_empty_environment)
{
  std::shared_ptr<mock::plant_t> plant{ std::make_shared<mock::plant_t>() };
  prx::obstacle_loader_t obstacle_loader{ prx::obstacle_loader_t(prx::input_path + "/environments/empty.yaml") };
  std::vector<std::string> obstacles_names{ obstacle_loader.get_names() };
  std::vector<std::shared_ptr<prx::movable_object_t>> obstacles{ obstacle_loader.get_obstacles() };

  prx::collision_checking::pqp::system_checker_t<mock::plant_t> checker(plant, obstacles);

  for (int i = 0; i < 100; ++i)
  {
    mock::State x{ mock::State ::Random() * 10 };
    BOOST_REQUIRE(not checker.collision(x));
  }
}

BOOST_AUTO_TEST_CASE(test_simple_obstacle)
{
  std::shared_ptr<mock::plant_t> plant{ std::make_shared<mock::plant_t>() };

  std::vector<std::shared_ptr<prx::movable_object_t>> obstacles;
  Eigen::Transform<double, 3, Eigen::Isometry> obstacle_pose;
  obstacle_pose.linear() = Eigen::Matrix3d::Identity();
  obstacle_pose.translation() = Eigen::Vector3d(0, 0, 0);
  obstacles.push_back(std::make_shared<prx::sphere_t>("o1", 0.5, obstacle_pose));
  // obstacles.back()->generate_collision_geometry();
  prx::collision_checking::pqp::system_checker_t<mock::plant_t> checker(plant, obstacles);

  mock::State x0{ 0.1, 0., 0. };
  BOOST_REQUIRE(checker.collision(x0));
  mock::State x1{ 10, 0., 0. };
  BOOST_REQUIRE(not checker.collision(x1));
  mock::State x2{ 0.5, 0., 0. };
  BOOST_REQUIRE(checker.collision(x2));
}

BOOST_AUTO_TEST_CASE(test_multiple_obstacles)
{
  std::shared_ptr<mock::plant_t> plant{ std::make_shared<mock::plant_t>() };

  std::vector<std::shared_ptr<prx::movable_object_t>> obstacles;
  Eigen::Transform<double, 3, Eigen::Isometry> obstacle_pose;

  obstacle_pose.linear() = Eigen::Matrix3d::Identity();
  obstacle_pose.translation() = Eigen::Vector3d(0, 0, 0);
  obstacles.push_back(std::make_shared<prx::sphere_t>("o1", 0.5, obstacle_pose));

  obstacle_pose.linear() = Eigen::Matrix3d::Identity();
  obstacle_pose.translation() = Eigen::Vector3d(1.0, 0, 0);
  obstacles.push_back(std::make_shared<prx::box_t>("o1", 0.5, 0.5, 0.5, obstacle_pose));
  // obstacles.back()->generate_collision_geometry();
  prx::collision_checking::pqp::system_checker_t<mock::plant_t> checker(plant, obstacles);

  mock::State x0{ 0.1, 0., 0. };
  BOOST_REQUIRE(checker.collision(x0));
  mock::State x1{ 1.5, 0., 0. };
  BOOST_REQUIRE(checker.collision(x1));
  mock::State x2{ 10, 0., 0. };
  BOOST_REQUIRE(not checker.collision(x2));
}
