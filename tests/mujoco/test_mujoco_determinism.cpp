#define BOOST_AUTO_TEST_MAIN mujoco_determinism_test
#include <boost/test/unit_test.hpp>

#include "prx/mujoco/mj_simulator.hpp"
#include "prx/simulation/playback/plan.hpp"

BOOST_AUTO_TEST_CASE(mushr_determinism_test)
{
  std::shared_ptr<prx::mujoco_simulator_t> sim = std::make_shared<prx::mujoco_simulator_t>("mushr/mushr.xml");
  sim->init_simulator();

  auto context = sim->get_context("mujoco");
  auto ss = context.first->get_state_space();
  auto cs = context.first->get_control_space();

  prx::init_random(111093);
  prx::space_point_t start = ss->make_point();
  prx::space_point_t end = ss->make_point();
  ss->copy_from_point(start);
  for (int i = 0; i < 100; i++)
  {
    sim->step_simulation();
  }
  std::cout << ss->print_point(start, 4) << std::endl;

  prx::plan_t plan(cs);
  for (int i = 0; i < 10; i++)
  {
    plan.append_onto_back(1.0);
    cs->sample(plan.back().control);
  }
  std::cout << plan.print(16) << std::endl;

  std::vector<prx::space_point_t> end_states;
  for (int i = 0; i < 30; i++)
  {
    context.first->propagate(start, plan, end);
    end_states.push_back(ss->clone_point(end));
    printf("%s\n", ss->print_point(end, 16).c_str());
  }

  // Compare the end states
  for (int i = 0; i < end_states.size(); i++)
  {
    for (int j = 0; j < end_states.size(); j++)
    {
      if (i == j)
        continue;
      for (int k = 0; k < ss->get_dimension(); k++)
      {
        BOOST_CHECK_CLOSE(end_states[i]->at(k), end_states[j]->at(k), 1e-8);
      }
    }
  }
}