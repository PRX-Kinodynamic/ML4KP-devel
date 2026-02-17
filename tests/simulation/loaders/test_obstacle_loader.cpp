#include "general/constants.hpp"
#define BOOST_AUTO_TEST_MAIN integrators
#include <string>
#include <boost/test/unit_test.hpp>

#include "prx/simulation/loaders/obstacle_loader.hpp"

BOOST_AUTO_TEST_CASE(test_empty_environment)
{
  // const std::string "/Users/Gary/pracsys/ML4KP-devel/resources/input_files/environments/empty.yaml"
  prx::obstacle_loader_t obstacle_loader = prx::obstacle_loader_t(prx::input_path + "/environments/empty.yaml");
  std::vector<std::string> obstacles_names = obstacle_loader.get_names();
  std::vector<std::shared_ptr<prx::movable_object_t>> obstacles = obstacle_loader.get_obstacles();

  BOOST_CHECK(obstacles_names.size() == 0);
  BOOST_CHECK(obstacles.size() == 0);
}

BOOST_AUTO_TEST_CASE(test_simple_obstacle)
{
  prx::obstacle_loader_t obstacle_loader =
      prx::obstacle_loader_t(prx::input_path + "/environments/simple_obstacle.yaml");
  std::vector<std::string> obstacles_names = obstacle_loader.get_names();
  std::vector<std::shared_ptr<prx::movable_object_t>> obstacles = obstacle_loader.get_obstacles();

  BOOST_CHECK(obstacles_names[0] == "simple_obstacle");
  BOOST_CHECK(obstacles.size() >= 1);
}

BOOST_AUTO_TEST_CASE(test_empty_environment_from_param)
{
  prx::param_loader pl(prx::input_path + "/environments/empty.yaml");

  prx::obstacle_loader_t obstacle_loader{ prx::obstacle_loader_t(pl) };
  std::vector<std::string> obstacles_names = obstacle_loader.get_names();
  std::vector<std::shared_ptr<prx::movable_object_t>> obstacles = obstacle_loader.get_obstacles();

  BOOST_CHECK(obstacles_names.size() == 0);
  BOOST_CHECK(obstacles.size() == 0);
}