#define BOOST_AUTO_TEST_MAIN integrators
#include <string>
#include <boost/test/unit_test.hpp>

#include "prx/simulation/loaders/obstacle_loader.hpp"

// Using acrobot due to its trajectories diverging fasts
// when changing integration step

BOOST_AUTO_TEST_CASE(test_empty_environment)
{
  prx::obstacle_loader_t obstacle_loader = prx::obstacle_loader_t("environments/empty.yaml");
  std::vector<std::string> obstacles_names = obstacle_loader.get_names();
  std::vector<std::shared_ptr<prx::movable_object_t>> obstacles = obstacle_loader.get_obstacles();

  BOOST_CHECK(obstacles_names.size() == 0);
  BOOST_CHECK(obstacles.size() == 0);
}

BOOST_AUTO_TEST_CASE(test_simple_obstacle)
{
  prx::obstacle_loader_t obstacle_loader = prx::obstacle_loader_t("environments/simple_obstacle.yaml");
  std::vector<std::string> obstacles_names = obstacle_loader.get_names();
  std::vector<std::shared_ptr<prx::movable_object_t>> obstacles = obstacle_loader.get_obstacles();

  BOOST_CHECK(obstacles_names[0] == "simple_obstacle");
  BOOST_CHECK(obstacles.size() == 1);
}

BOOST_AUTO_TEST_CASE(test_obj_obstacle)
{
  prx::obstacle_loader_t obstacle_loader = prx::obstacle_loader_t("environments/obstacle_obj.yaml");
  std::vector<std::string> obstacles_names = obstacle_loader.get_names();
  std::vector<std::shared_ptr<prx::movable_object_t>> obstacles = obstacle_loader.get_obstacles();

  BOOST_CHECK(obstacles_names[0] == "test_hole");
  BOOST_CHECK(obstacles.size() == 1);
}