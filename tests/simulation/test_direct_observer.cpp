#define BOOST_AUTO_TEST_MAIN velocity_observer_test

#include <boost/test/unit_test.hpp>
#include <string>

#include "prx/simulation/observers/direct_observer.hpp"

using prx::simulation::direct_observer_t;
using prx::simulation::observer_t;

struct test_plant_t : prx::plant_t
{
  test_plant_t() : prx::plant_t("test_plant")
  {
    x = y = z = 0;
    dx = dy = dz = 0;
    ddx = ddy = ddz = 0;
    ax = ay = az = 0;
    wx = wy = wz = 0;
    dwx = dwy = dwz = 0;

    state_memory = { &x, &y, &z, &ax, &ay, &az, &dx, &dy, &dz, &wx, &wy, &wz };
    derivative_memory = { &dx, &dy, &dz, &wx, &wy, &wz, &ddx, &ddy, &ddz, &dwx, &dwy, &dwz };
    state_space = new prx::space_t("EEERRREEEEEE", state_memory, "plant_state");
    derivative_space = new prx::space_t("EEEEEEEEEEEE", derivative_memory, "plant_deriv_state");
  }
  virtual void update_configuration(){};
  virtual void compute_derivative(){};

  double x, y, z;        // positions
  double dx, dy, dz;     // vel = d*
  double ddx, ddy, ddz;  // accel = dd*

  // Conviniently, only quaternion and angular accelerations
  double ax, ay, az;     // orientation, not using quat for convinience/easiness. angle * = a*
  double wx, wy, wz;     // angular vel = w*
  double dwx, dwy, dwz;  // angular accel = dw*
};

BOOST_AUTO_TEST_CASE(observer_observes_position)
{
  // Given the plant, observe "x"
  std::shared_ptr<test_plant_t> plant = std::make_shared<test_plant_t>();
  std::vector<double*> addresses{ &plant->x, &plant->y };
  direct_observer_t observer(addresses, "EE");
  BOOST_CHECK_MESSAGE(observer.get_observed_space()->get_dimension() == 2, "Wrong observation size");
  prx::space_point_t observation = observer.get_observed_space()->make_point();

  const double x_0{ 5.0 };
  const double y_0{ 10.0 };
  plant->x = x_0;
  plant->y = y_0;

  observer(plant, observation);
  const double observation_x_0{ (*observation)[0] };
  const double observation_y_0{ (*observation)[1] };
  BOOST_CHECK_MESSAGE(observation_x_0 == x_0, "Wrong observation");
  BOOST_CHECK_MESSAGE(observation_y_0 == y_0, "Wrong observation");

  // Changing z shoudln't change observation
  plant->z = 100;
  observer(plant, observation);
  BOOST_CHECK_MESSAGE(observation_x_0 == x_0, "Wrong observation");
  BOOST_CHECK_MESSAGE(observation_y_0 == y_0, "Wrong observation");

  const double x_1{ 20.0 };
  const double y_1{ 10.0 };

  plant->x = x_1;
  plant->y = y_1;

  observer(plant, observation);
  const double observation_x_1{ (*observation)[0] };
  const double observation_y_1{ (*observation)[1] };

  BOOST_CHECK_MESSAGE(observation_x_1 == x_1, "Wrong observation");
  BOOST_CHECK_MESSAGE(observation_y_1 == y_1, "Wrong observation");
}

BOOST_AUTO_TEST_CASE(imu_like_observer)
{
  // Given the plant, observe "x"
  std::shared_ptr<test_plant_t> plant = std::make_shared<test_plant_t>();
  std::vector<double*> accel_addresses{ &plant->ddx, &plant->ddy, &plant->ddz };
  std::vector<double*> gyros_addresses{ &plant->dwx, &plant->dwy, &plant->dwz };

  observer_t observer{};
  // making explicit that are two observers. Could be ones
  observer
      .add(new direct_observer_t(accel_addresses, "EEE"))  // no-lint
      .add(new direct_observer_t(gyros_addresses, "EEE"));

  BOOST_CHECK_MESSAGE(observer.get_observed_space()->get_dimension() == 6, "Wrong observation size");
  prx::space_point_t observation = observer.get_observed_space()->make_point();

  const double ddxyz_0{ 0.5 };
  const double dwxyz_0{ 0.2 };

  plant->ddx = ddxyz_0;
  plant->ddy = ddxyz_0;
  plant->ddz = ddxyz_0;

  plant->dwx = dwxyz_0;
  plant->dwy = dwxyz_0;
  plant->dwz = dwxyz_0;

  observer(plant, observation);
  const double observation_ddx_0{ (*observation)[0] };
  const double observation_ddy_0{ (*observation)[1] };
  const double observation_ddz_0{ (*observation)[2] };

  const double observation_dwx_0{ (*observation)[3] };
  const double observation_dwy_0{ (*observation)[4] };
  const double observation_dwz_0{ (*observation)[5] };

  BOOST_CHECK_MESSAGE(observation_ddx_0 == ddxyz_0, "Wrong observation");
  BOOST_CHECK_MESSAGE(observation_ddy_0 == ddxyz_0, "Wrong observation");
  BOOST_CHECK_MESSAGE(observation_ddz_0 == ddxyz_0, "Wrong observation");

  BOOST_CHECK_MESSAGE(observation_dwx_0 == dwxyz_0, "Wrong observation");
  BOOST_CHECK_MESSAGE(observation_dwy_0 == dwxyz_0, "Wrong observation");
  BOOST_CHECK_MESSAGE(observation_dwz_0 == dwxyz_0, "Wrong observation");
}
