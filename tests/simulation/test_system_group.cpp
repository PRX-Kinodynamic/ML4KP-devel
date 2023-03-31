#define BOOST_AUTO_TEST_MAIN system_group_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/simulation/simulator.hpp"
#include "prx/simulation/plants/plants.hpp"
namespace sg_test
{
struct test_simulator : prx::simulator_t
{
  test_simulator(prx::system_ptr_t sys_ptr) : simulator_t(prx::plant_type::ANALYTICAL)
  {
    prx::simulation_step = 0.01;
    this->systems["test_system"] = sys_ptr;
  };
  virtual void step_simulation(prx::propagate_step step) override
  {
    for (auto s : this->systems)
    {
      s.second->propagate(prx::simulation_step, step);
      s.second->get_state_space()->enforce_bounds();
    }
  }
  virtual void reset_simulation() override{};
};

// A passive pendulum: (0,0) is pointing down.
struct passive_pendulum : public prx::plant_t
{
  passive_pendulum() : plant_t("passive_pendulum")
  {
    state_memory = { &_theta, &_thetadot };
    state_space = new prx::space_t("RE", state_memory, "pendulum_state");
    state_space->set_bounds({ -PRX_PI, -2 * PRX_PI }, { PRX_PI, 2 * PRX_PI });
    input_control_space = new prx::space_t("", control_memory, "Passive");
    derivative_memory = { &_thetadot, &_thetadotdot };
    derivative_space = new prx::space_t("EE", derivative_memory, "pendulum_derivative");
    set_integrator(prx::integrator_t::kRK4);
  };
  void propagate(const double simulation_step)
  {
    integrator->integrate(simulation_step);
  }
  void update_configuration()
  {
  }
  void compute_derivative()
  {
    _thetadotdot = gravity / length * std::sin(_theta + PRX_PI) - friction / (mass * length * length) * _thetadot;
  }
  double _theta, _thetadot, _thetadotdot;
  const double gravity{ 9.81 };
  const double length{ 0.5 };
  const double friction{ 0.1 };
  const double mass{ 0.15 };
};
}  // namespace sg_test

BOOST_AUTO_TEST_CASE(builds_correctly)
{
  std::shared_ptr<sg_test::passive_pendulum> pendulum(new sg_test::passive_pendulum());
  prx::system_group_t system_group({ pendulum }, prx::plant_type::ANALYTICAL);
  sg_test::test_simulator simulator(pendulum);
  system_group.set_simulator(&simulator);
  BOOST_CHECK_MESSAGE(system_group.get_state_space() != nullptr, "System group's state space wasn't build");
  BOOST_CHECK_MESSAGE(system_group.get_control_space() != nullptr, "System group's control space wasn't build");
  BOOST_CHECK_MESSAGE(system_group.get_parameter_space() != nullptr, "System group's parameter space wasn't build");
}

BOOST_AUTO_TEST_CASE(propagate_once_test)
{
  std::shared_ptr<sg_test::passive_pendulum> pendulum(new sg_test::passive_pendulum());
  prx::system_group_t system_group({ pendulum }, prx::plant_type::ANALYTICAL);
  sg_test::test_simulator simulator(pendulum);
  system_group.set_simulator(&simulator);

  const std::vector<double> start_state = { 0.1, 0 };
  const std::vector<double> control = {};  // Any value will work... is a passive pendulum
  // Initializing end state to a state out of bounds to ensure that propagate updates it.
  std::vector<double> end_state = { 10, 10 };
  system_group.propagate_once(start_state, control, end_state);

  BOOST_CHECK_MESSAGE(end_state[0] != 10, "Got: " << end_state[0]);
  BOOST_CHECK_MESSAGE(end_state[1] != 10, "Got: " << end_state[1]);

  // The state is close to (0,0)
  BOOST_CHECK_MESSAGE(-0.1 < end_state[0] && end_state[0] < +0.1, "Got: " << end_state[0]);
}

BOOST_AUTO_TEST_CASE(propagate_start_control_duration_endstate_test)
{
  std::shared_ptr<sg_test::passive_pendulum> pendulum(new sg_test::passive_pendulum());
  prx::system_group_t system_group({ pendulum }, prx::plant_type::ANALYTICAL);
  sg_test::test_simulator simulator(pendulum);
  system_group.set_simulator(&simulator);

  const double duration{ 10.0 };
  const std::vector<double> start_state = { 0.1, 0 };
  const std::vector<double> control = {};  // Any value will work... is a passive pendulum
  // Initializing end state to a state out of bounds to ensure that propagate updates it.
  std::vector<double> end_state = { 10, 10 };
  system_group.propagate(start_state, control, duration, end_state);

  const std::vector<double> expected_end_state = { 0.0, 0.0 };
  BOOST_CHECK_SMALL(end_state[0] - expected_end_state[0], 1e-3);
  BOOST_CHECK_SMALL(end_state[1] - expected_end_state[1], 1e-3);
}

BOOST_AUTO_TEST_CASE(propagate_start_plan_endstate_test)
{
  std::shared_ptr<sg_test::passive_pendulum> pendulum(new sg_test::passive_pendulum());
  prx::system_group_t system_group({ pendulum }, prx::plant_type::ANALYTICAL);
  sg_test::test_simulator simulator(pendulum);
  system_group.set_simulator(&simulator);

  const std::vector<double> start_state = { 0.1, 0 };
  prx::plan_t plan(system_group.get_control_space());
  const std::vector<double> control = {};  // Any value will work... is a passive pendulum
  plan.copy_onto_back(control, 10);
  // Initializing end state to a state out of bounds to ensure that propagate updates it.
  std::vector<double> end_state = { 10, 10 };
  system_group.propagate(start_state, plan, end_state);

  const std::vector<double> expected_end_state = { 0.0, 0.0 };
  BOOST_CHECK_SMALL(end_state[0] - expected_end_state[0], 1e-3);
  BOOST_CHECK_SMALL(end_state[1] - expected_end_state[1], 1e-3);
}