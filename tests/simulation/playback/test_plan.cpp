#define BOOST_AUTO_TEST_MAIN constants_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/simulation/playback/plan.hpp"

namespace mock
{
struct plan_space_test_t
{
  plan_space_test_t() : u0(0), u1(0), address({ &u0, &u1 }), space("EE", address, "space_test")
  {
  }
  double u0, u1;
  std::vector<double*> address;
  prx::space_t space;
};
}  // namespace mock

BOOST_AUTO_TEST_CASE(plan_construction_test)
{
  mock::plan_space_test_t test;
  prx::space_t& space(test.space);
  prx::plan_t plan(&space);

  BOOST_REQUIRE(plan.size() == 0);
}

BOOST_AUTO_TEST_CASE(plan_resize_test)
{
  mock::plan_space_test_t test;
  prx::space_t& space(test.space);
  prx::plan_t plan(&space);

  plan.resize(5);

  BOOST_REQUIRE(plan.size() == 5);

  std::size_t cont{ 0 };
  for (auto step : plan)
  {
    cont++;
  }
  BOOST_REQUIRE_MESSAGE(5 == cont, EXPECTED_GOT(5, cont));
}

BOOST_AUTO_TEST_CASE(plan_to_and_from_file_test)
{
  mock::plan_space_test_t test;
  prx::space_t& space(test.space);
  prx::plan_t plan_to(&space);
  prx::plan_t plan_from(&space);

  plan_to.resize(2);
  space.copy(plan_to[0].control, { 1.0, 1.0 });
  space.copy(plan_to[1].control, { 2.0, 2.0 });
  plan_to[0].duration = 1.0;
  plan_to[1].duration = 2.0;

  const std::string filename{ "/tmp/plan_test.txt" };
  plan_to.to_file(filename);
  plan_from.from_file(filename);

  BOOST_REQUIRE_MESSAGE(2 == plan_from.size(), EXPECTED_GOT(2, plan_from.size()));

  BOOST_REQUIRE_MESSAGE(1.0 == plan_from[0].control->at(0), EXPECTED_GOT(1.0, plan_from[0].control->at(0)));
  BOOST_REQUIRE_MESSAGE(1.0 == plan_from[0].control->at(1), EXPECTED_GOT(1.0, plan_from[0].control->at(1)));
  BOOST_REQUIRE_MESSAGE(1.0 == plan_from[0].duration, EXPECTED_GOT(1.0, plan_from[0].duration));

  BOOST_REQUIRE_MESSAGE(2.0 == plan_from[1].control->at(0), EXPECTED_GOT(2.0, plan_from[1].control->at(0)));
  BOOST_REQUIRE_MESSAGE(2.0 == plan_from[1].control->at(1), EXPECTED_GOT(2.0, plan_from[1].control->at(1)));
  BOOST_REQUIRE_MESSAGE(2.0 == plan_from[1].duration, EXPECTED_GOT(2.0, plan_from[1].duration));
}