#define BOOST_TEST_MODULE bang_bang_ctrls
#include <boost/test/included/unit_test.hpp>
#include <string>

#include "prx/simulation/playback/plan.hpp"

BOOST_AUTO_TEST_CASE(plan_copy_onto_back_works_correctly)
{
  prx::simulation_step = 0.01;
  double u0, u1;
  u0 = u1 = 1;
  std::vector<double*> address_1 = { &u0, &u1 };
  prx::space_t control_space("EE", address_1, "space_1");

  prx::plan_t plan(&control_space);
  prx::space_point_t cs_pt = control_space.make_point();

  const std::size_t plan_size{ 10 };

  for (int i = 0; i < plan_size; ++i)
  {
    (*cs_pt)[0] = i;
    (*cs_pt)[1] = i + 1;
    plan.copy_onto_back(cs_pt, prx::simulation_step);
  }

  for (std::size_t i = 0; i < plan_size; ++i)
  {
    BOOST_CHECK(plan[i].control->at(0) == i);
    BOOST_CHECK(plan[i].control->at(1) == i + 1);
    BOOST_CHECK(plan[i].duration == prx::simulation_step);
  }
}

BOOST_AUTO_TEST_CASE(plan_expands_correctly)
{
  prx::simulation_step = 0.01;
  double u0, u1;
  u0 = u1 = 1;
  std::vector<double*> address_1 = { &u0, &u1 };
  prx::space_t control_space("EE", address_1, "space_1");

  prx::plan_t plan(&control_space);
  prx::space_point_t cs_pt = control_space.make_point();

  const std::size_t plan_size{ 10 };

  cs_pt->at(0) = 1;
  cs_pt->at(1) = 2;
  plan.copy_onto_back(cs_pt, 1);

  plan.expand();
  for (std::size_t i = 0; i < plan_size; ++i)
  {
    BOOST_CHECK(plan[i].control->at(0) == 1);
    BOOST_CHECK(plan[i].control->at(1) == 2);
    BOOST_CHECK(plan[i].duration == prx::simulation_step);
  }
}