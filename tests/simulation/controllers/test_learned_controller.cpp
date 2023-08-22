#ifndef TORCH_NOT_BUILT
#define BOOST_AUTO_TEST_MAIN learned_controller_test
#include "prx/simulation/controllers/learned_controller.hpp"
#include <boost/test/unit_test.hpp>
#include "prx/simulation/plants/plants.hpp"
#include "prx/mujoco/mj_simulator.hpp"

BOOST_AUTO_TEST_CASE(learned_controller_load_test)
{
  std::string controller_params_file = "controllers/trailer_car_learned.yaml";

  std::string plant_name = "trailer_car";
  std::string plant_path = "trailer_car";
  auto plant = prx::system_factory_t::create_system(plant_name, plant_path);

  prx::learned_controller_t controller(plant, controller_params_file);
}

BOOST_AUTO_TEST_CASE(learned_controller_trailer_car_test)
{
  std::string controller_params_file = "controllers/trailer_car_learned.yaml";

  std::string plant_name = "trailer_car";
  std::string plant_path = "trailer_car";
  auto plant = prx::system_factory_t::create_system(plant_name, plant_path);

  prx::learned_controller_t controller(plant, controller_params_file);

  prx::space_point_t goal_state = plant->get_state_space()->make_point();
  std::vector<double> goal_state_vec = {5., 0., 0., 0.};
  plant->get_state_space()->copy(goal_state, goal_state_vec);
  controller.set_goal(goal_state);

  controller.compute_controls();

  BOOST_CHECK(prx::are_approx_equal(controller.get_control_space()->at(0), 0.452, 1e-2));
  BOOST_CHECK(prx::are_approx_equal(controller.get_control_space()->at(1),-0.027, 1e-2));
}

BOOST_AUTO_TEST_CASE(learned_controller_mushr_test)
{
  std::shared_ptr<prx::mujoco_simulator_t> sim = std::make_shared<prx::mujoco_simulator_t>("mushr/mushr.xml");
  sim->init_simulator();

  auto context = sim->get_context("mujoco");
  auto ss = context.first->get_state_space();
  for (int i = 0; i < 100; i++)
  {
    sim->step_simulation();
  }

  std::string controller_params_file = "controllers/mushr_full_state_learned.yaml";

  prx::system_ptr_t mj_plant = context.first->get_primary_system();
  prx::learned_controller_t controller(mj_plant, controller_params_file);

  prx::space_point_t goal_state = controller.get_state_space()->make_point();
  goal_state -> at(0) = 5.0;
  goal_state -> at(1) = 0.0;
  goal_state -> at(3) = 1.0;
  controller.set_goal(goal_state);

  controller.compute_controls();
  
  BOOST_CHECK(prx::are_approx_equal(controller.get_control_space()->at(0), 0.0424, 1e-2));
  BOOST_CHECK(prx::are_approx_equal(controller.get_control_space()->at(1), 0.9373, 1e-2));
}

#else
int main()
{
  return -1;
}
#endif