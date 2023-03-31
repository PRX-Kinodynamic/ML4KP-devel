#define BOOST_AUTO_TEST_MAIN condition_check_test
#include <string>
#include <boost/test/unit_test.hpp>

#include "prx/utilities/spaces/space.hpp"
#include "prx/planning/condition_check.hpp"
#include "prx/simulation/playback/plan.hpp"
#include "prx/simulation/playback/trajectory.hpp"
#include "prx/planning/planner_functions/planner_functions.hpp"

BOOST_AUTO_TEST_CASE(condition_check_test)
{
  int iters = 1e3;
  prx::condition_check_t check_1("iterations", iters);
  prx::simulation_step = 0.01;

  int iters_test = 0;
  do
  {
    iters_test++;
  } while (!check_1.check());

  BOOST_CHECK(iters_test == iters);

  double x, y, z;
  x = y = z = 1;
  std::vector<double*> address_1 = { &x, &y, &z };
  prx::space_t space_1("EEE", address_1, "space_1");

  prx::space_point_t goal = space_1.make_point();
  prx::space_point_t pt = space_1.make_point();
  prx::space_point_t pt_aux = space_1.make_point();
  space_1.copy(goal, { 1, 1, 1 });

  // prx::custom_check_t cc = [&]()
  // {
  // 	// space_1.copy_to_point(pt);
  // // std::cout << "euclidean_2d: " << prx::space_t::euclidean_2d(pt, goal, 0, 3)  << std::endl;
  // 	return prx::space_t::euclidean_distance(&space_1, goal) < 0.1;
  // 	// return prx::space_t::euclidean_2d(pt, goal, 0, 3) < 0.1;
  // };

  auto cc = create_default_goal_check(&space_1, goal, 0.1);

  space_1.copy(pt_aux, { 0, 0, 0 });
  prx::condition_check_t check_2(cc);
  check_1.reset();
  check_1.add_condition(&check_2);
  iters_test = 0;
  do
  {
    (*pt_aux)[0] += 0.1;
    (*pt_aux)[1] += 0.1;
    (*pt_aux)[2] += 0.1;
    space_1.copy_from_point(pt_aux);
    iters_test++;
  } while (!check_1.check());

  std::cout << "pt_aux: " << pt_aux << std::endl;
  std::cout << "iters_test: " << iters_test << std::endl;

  BOOST_CHECK(iters_test < iters);
  BOOST_CHECK(prx::space_t::euclidean_2d(pt_aux, goal, 0, 3) < 0.1);
}
