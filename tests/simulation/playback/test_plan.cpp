#define BOOST_TEST_MODULE plan_test
#include <string>
#include <boost/test/included/unit_test.hpp>

#include "prx/simulation/playback/plan.hpp"

using prx::plan_t;
struct plant_3d_test
{
  double x, y, theta;
  std::vector<double*> addresses;
  prx::space_t* space;
  plant_3d_test()
  {
    x = y = theta = 1;
    addresses = { &x, &y, &theta };
    space = new prx::space_t("EER", addresses, "space_3d");
  }
};

BOOST_AUTO_TEST_CASE(build_from_space)
{
  plant_3d_test plant{};
  plan_t plan(plant.space);
  BOOST_CHECK(plan.size() == 0);
}