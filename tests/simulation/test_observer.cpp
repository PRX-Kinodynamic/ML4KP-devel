#define BOOST_AUTO_TEST_MAIN observer_test

#include <boost/test/unit_test.hpp>
#include <string>

#include "prx/simulation/observer.hpp"

using prx::simulation::observer_t;

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
struct passthrough_observer_t : public observer_t
{
  passthrough_observer_t(const std::size_t dim_to_observe) : observer_t(), _dim_to_observe(dim_to_observe)
  {
    memory = 0;
    _observer_memory = { &memory };
    _observed_space = new prx::space_t("E", _observer_memory, "passthrough_observer_space");
  }
  virtual void observe(const std::shared_ptr<prx::plant_t> plant) override final
  {
    memory = plant->get_state_space()->at(_dim_to_observe);
  }

  double memory;
  const std::size_t _dim_to_observe;
};

BOOST_AUTO_TEST_CASE(observer_observes)
{
  // Given the plant, observe "x"
  std::shared_ptr<test_plant_t> plant = std::make_shared<test_plant_t>();
  const std::size_t x_idx{ 0 };
  passthrough_observer_t observer(x_idx);

  BOOST_CHECK_MESSAGE(observer.get_observed_space()->get_dimension() == 1, "Wrong observation size");
  prx::space_point_t observation = observer.get_observed_space()->make_point();

  plant->x = 10;
  observer(plant, observation);
  BOOST_CHECK_MESSAGE((*observation)[x_idx] == 10, "Wrong observation");
  plant->y = 100;
  plant->z = 100;
  observer(plant, observation);
  BOOST_CHECK_MESSAGE((*observation)[x_idx] == 10, "Wrong observation");
  plant->x = 100;
  observer(plant, observation);
  BOOST_CHECK_MESSAGE((*observation)[x_idx] == 100, "Wrong observation");
}

BOOST_AUTO_TEST_CASE(observer_adds_one_observer)
{
  // Given the plant, observe "x"
  std::shared_ptr<test_plant_t> plant = std::make_shared<test_plant_t>();
  observer_t observer{};
  const std::size_t x_idx{ 0 };
  observer.add(new passthrough_observer_t(x_idx));
  BOOST_CHECK_MESSAGE(observer.get_observed_space()->get_dimension() == 1, "Wrong observation size");
  prx::space_point_t observation = observer.get_observed_space()->make_point();

  plant->x = 10;
  observer(plant, observation);
  BOOST_CHECK_MESSAGE((*observation)[x_idx] == 10, "Wrong observation");
  plant->y = 100;
  plant->z = 100;
  observer(plant, observation);
  BOOST_CHECK_MESSAGE((*observation)[x_idx] == 10, "Wrong observation");
  plant->x = 100;
  observer(plant, observation);
  BOOST_CHECK_MESSAGE((*observation)[x_idx] == 100, "Wrong observation");
}

BOOST_AUTO_TEST_CASE(observer_adds_multiple_observers)
{
  // Given the plant, observe "x"
  std::shared_ptr<test_plant_t> plant = std::make_shared<test_plant_t>();
  observer_t observer{};
  const std::size_t x_idx{ 0 };
  const std::size_t y_idx{ 1 };
  const std::size_t z_idx{ 2 };
  const std::size_t expected_observation_dim{ 3 };
  observer
      .add(new passthrough_observer_t(x_idx))  // no-lint
      .add(new passthrough_observer_t(y_idx))  // no-lint
      .add(new passthrough_observer_t(z_idx));

  const std::size_t observation_dim{ observer.get_observed_space()->get_dimension() };
  BOOST_CHECK_MESSAGE(observation_dim == expected_observation_dim,
                      "Wrong observation size. Expected: " << expected_observation_dim << " got: " << observation_dim);
  prx::space_point_t observation = observer.get_observed_space()->make_point();

  plant->x = 10;
  plant->y = 20;
  plant->z = 30;
  observer(plant, observation);
  BOOST_CHECK_MESSAGE((*observation)[x_idx] == 10, "Wrong observation");
  BOOST_CHECK_MESSAGE((*observation)[y_idx] == 20, "Wrong observation");
  BOOST_CHECK_MESSAGE((*observation)[z_idx] == 30, "Wrong observation");

  // Change only one variable, remaining should remain the same
  plant->y = 200;
  observer(plant, observation);
  BOOST_CHECK_MESSAGE((*observation)[x_idx] == 10, "Wrong observation");
  BOOST_CHECK_MESSAGE((*observation)[y_idx] == 200, "Wrong observation");
  BOOST_CHECK_MESSAGE((*observation)[z_idx] == 30, "Wrong observation");

  // Change the remaining variables
  plant->x = 100;
  plant->z = 300;
  observer(plant, observation);
  BOOST_CHECK_MESSAGE((*observation)[x_idx] == 100, "Wrong observation");
  BOOST_CHECK_MESSAGE((*observation)[y_idx] == 200, "Wrong observation");
  BOOST_CHECK_MESSAGE((*observation)[z_idx] == 300, "Wrong observation");
}