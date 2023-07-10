#define BOOST_AUTO_TEST_MAIN velocity_observer_test

#include <boost/test/unit_test.hpp>
#include <string>

#include "prx/simulation/observers/velocity_observer.hpp"

using prx::simulation::velocity_observer_t;

struct test_plant_t : prx::plant_t
{
  test_plant_t() : prx::plant_t("test_plant")
  {
    x = y = z = 0;
    state_memory = { &x, &y, &z };
    state_space = new prx::space_t("EEE", state_memory, "plant_state");
  }
  virtual void update_configuration(){};
  virtual void compute_derivative(){};

  double x, y, z;
};

BOOST_AUTO_TEST_CASE(observer_observes_2dvelocity)
{
  // Given the plant, observe "x"
  std::shared_ptr<test_plant_t> plant = std::make_shared<test_plant_t>();
  std::vector<double*> addresses{ &plant->x, &plant->y };
  velocity_observer_t observer(addresses);
  BOOST_CHECK_MESSAGE(observer.get_observed_space()->get_dimension() == 1, "Wrong observation size");
  prx::space_point_t observation = observer.get_observed_space()->make_point();

  const double velocity_xy_0{ 10.0 };
  const double expected_velocity_0{ std::sqrt(2.0 * velocity_xy_0 * velocity_xy_0) };
  plant->x = velocity_xy_0;
  plant->y = velocity_xy_0;

  observer(plant, observation);

  const double observation_0{ (*observation)[0] };
  BOOST_CHECK_MESSAGE(observation_0 == expected_velocity_0,
                      "Wrong observation. Got: " << observation_0 << " expected: " << expected_velocity_0);

  // Changing z shoudln't change observation
  plant->z = 100;
  observer(plant, observation);
  BOOST_CHECK_MESSAGE((*observation)[0] == expected_velocity_0, "Wrong observation");

  const double velocity_x_1{ 20.0 };
  const double velocity_y_1{ 10.0 };
  const double expected_velocity_1{ std::sqrt(velocity_x_1 * velocity_x_1 + velocity_y_1 * velocity_y_1) };
  plant->x = velocity_x_1;
  plant->y = velocity_y_1;

  observer(plant, observation);
  BOOST_CHECK_MESSAGE((*observation)[0] == expected_velocity_1, "Wrong observation");
}

BOOST_AUTO_TEST_CASE(observer_observes_3dvelocity)
{
  // Given the plant, observe "x"
  std::shared_ptr<test_plant_t> plant = std::make_shared<test_plant_t>();
  std::vector<double*> addresses{ &plant->x, &plant->y, &plant->z };
  velocity_observer_t observer(addresses);
  BOOST_CHECK_MESSAGE(observer.get_observed_space()->get_dimension() == 1, "Wrong observation size");
  prx::space_point_t observation = observer.get_observed_space()->make_point();

  const double velocity_xyz_0{ 10.0 };
  const double expected_velocity_0{ std::sqrt(3.0 * velocity_xyz_0 * velocity_xyz_0) };
  plant->x = velocity_xyz_0;
  plant->y = velocity_xyz_0;
  plant->z = velocity_xyz_0;

  observer(plant, observation);

  const double observation_0{ (*observation)[0] };
  BOOST_CHECK_MESSAGE(observation_0 == expected_velocity_0,
                      "Wrong observation. Got: " << observation_0 << " expected: " << expected_velocity_0);

  // Changing z changes the observation
  const double velocity_z_1{ 30.0 };
  plant->z = velocity_z_1;
  const double expected_velocity_1{ std::sqrt(2.0 * velocity_xyz_0 * velocity_xyz_0 + velocity_z_1 * velocity_z_1) };
  observer(plant, observation);
  BOOST_CHECK_MESSAGE((*observation)[0] == expected_velocity_1, "Wrong observation");

  const double velocity_x_2{ 20.0 };
  const double velocity_y_2{ 30.0 };
  const double velocity_z_2{ 40.0 };
  const double expected_velocity_2{ std::sqrt(velocity_x_2 * velocity_x_2 + velocity_y_2 * velocity_y_2 +
                                              velocity_z_2 * velocity_z_2) };
  plant->x = velocity_x_2;
  plant->y = velocity_y_2;
  plant->z = velocity_z_2;

  observer(plant, observation);
  BOOST_CHECK_MESSAGE((*observation)[0] == expected_velocity_2, "Wrong observation");
}