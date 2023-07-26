#define BOOST_AUTO_TEST_MAIN mujoco_playback_test
#include <boost/test/unit_test.hpp>

#include "prx/mujoco/mj_simulator.hpp"
#include "prx/simulation/playback/plan.hpp"

BOOST_AUTO_TEST_CASE(mushr_without_obstacles_test)
{
  std::shared_ptr<prx::mujoco_simulator_t> sim = std::make_shared<prx::mujoco_simulator_t>("mushr/mushr.xml");
  sim->init_simulator();

  auto context = sim->get_context("mujoco");
  auto ss = context.first->get_state_space();
  auto cs = context.first->get_control_space();

  const std::vector<std::vector<double>> controls = {
    { -0.5924146691227876, 0.8764427682900211 }, { 0.4031970273566428, 0.3431110600969522 },
    { -0.7296366493093971, 0.9766263000094801 }, { -0.7157116496186970, -0.8685083506989437 },
    { -0.9605540758987892, 0.6471479997437821 }, { -0.5270037870717759, 0.1567385116138973 },
    { -0.8578774143256882, 0.2097287678646542 }, { -0.5171879314187702, 0.0587042512258695 },
    { 0.8700072523627274, -0.3150636282680475 }, { 0.5755902430004103, -0.6543351021042898 },
  };

  prx::space_point_t start = ss->make_point();
  prx::space_point_t end = ss->make_point();
  ss->copy_from_point(start);
  for (int i = 0; i < 100; i++)
  {
    sim->step_simulation();
  }

  prx::plan_t plan(cs);
  for (int i = 0; i < 10; i++)
  {
    plan.append_onto_back(1.0);
    cs->copy_point_from_vector(plan.back().control, controls[i]);
  }

  const std::vector<double> end_state = {
    1.4784779467791176,   -0.8461897798823226, -0.0003558931852912,  0.1819573534935469,  -0.0030885777163593,
    -0.0002765774032239,  -0.9833015334587120, 0.4663985527712488,   0.4100059617223481,  36.8663604795329221,
    0.3844847650042804,   25.5448825900043914, 35.0979167559352021,  22.0050015832041836, 0.4851152183849354,
    0.3303899319226496,   0.0014526712414283,  -0.0352929874627816,  -0.0037699710487359, -0.8006621362108777,
    0.0008576656082535,   0.0056437022587917,  -10.8356434615856685, 0.0007136892593291,  -14.1982004481014723,
    -10.0291708212601023, -13.8347511856679031
  };
  context.first->propagate(start, plan, end);
  for (int i = 0; i < ss->get_dimension(); i++)
  {
    BOOST_CHECK_CLOSE(end->at(i), end_state[i], 1e-8);
  }
}