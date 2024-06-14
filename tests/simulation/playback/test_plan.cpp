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

BOOST_AUTO_TEST_CASE(plan_duration_test)
{
  mock::plan_space_test_t test;
  prx::space_t& space(test.space);
  prx::plan_t plan(&space);

  plan.copy_onto_back(Eigen::Vector2d(0, 0), 1);
  plan.copy_onto_back(Eigen::Vector2d(1, 1), 1);
  plan.copy_onto_back(Eigen::Vector2d(2, 2), 1);

  const int expected_duration{ 3 };
  BOOST_REQUIRE_MESSAGE(plan.duration() == expected_duration, EXPECTED_GOT(expected_duration, plan.duration()));
}

BOOST_AUTO_TEST_CASE(plan_at_test)
{
  mock::plan_space_test_t test;
  prx::space_t& space(test.space);
  prx::plan_t plan(&space);

  plan.copy_onto_back(Eigen::Vector2d(0, 0), 1);
  plan.copy_onto_back(Eigen::Vector2d(1, 1), 1);
  plan.copy_onto_back(Eigen::Vector2d(2, 2), 1);

  const Eigen::Vector2d expected_t0p0{ 0, 0 };
  const Eigen::Vector2d expected_t0p5{ 0, 0 };
  const Eigen::Vector2d expected_t1p0{ 1, 1 };
  const Eigen::Vector2d expected_t1p5{ 1, 1 };
  const Eigen::Vector2d expected_t2p0{ 2, 2 };
  const Eigen::Vector2d expected_t2p5{ 2, 2 };

  const Eigen::Vector2d point_at_t0p0{ Vec(plan.at(0.0)) };
  const Eigen::Vector2d point_at_t0p5{ Vec(plan.at(0.5)) };
  const Eigen::Vector2d point_at_t1p0{ Vec(plan.at(1.0)) };
  const Eigen::Vector2d point_at_t1p5{ Vec(plan.at(1.5)) };
  const Eigen::Vector2d point_at_t2p0{ Vec(plan.at(2.0)) };
  const Eigen::Vector2d point_at_t2p5{ Vec(plan.at(2.5)) };

  BOOST_REQUIRE_MESSAGE(expected_t0p0.isApprox(point_at_t0p0), EXPECTED_GOT(expected_t0p0, point_at_t0p0));
  BOOST_REQUIRE_MESSAGE(expected_t0p5.isApprox(point_at_t0p5), EXPECTED_GOT(expected_t0p5, point_at_t0p5));
  BOOST_REQUIRE_MESSAGE(expected_t1p0.isApprox(point_at_t1p0), EXPECTED_GOT(expected_t1p0, point_at_t1p0));
  BOOST_REQUIRE_MESSAGE(expected_t1p5.isApprox(point_at_t1p5), EXPECTED_GOT(expected_t1p5, point_at_t1p5));
  BOOST_REQUIRE_MESSAGE(expected_t2p0.isApprox(point_at_t2p0), EXPECTED_GOT(expected_t2p0, point_at_t2p0));
  BOOST_REQUIRE_MESSAGE(expected_t2p5.isApprox(point_at_t2p5), EXPECTED_GOT(expected_t2p5, point_at_t2p5));
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

BOOST_AUTO_TEST_CASE(plan_split_test)
{
  mock::plan_space_test_t test;
  prx::space_t& space(test.space);
  prx::plan_t plan_to_split(&space);

  Eigen::Vector2d u0(1.0, 1.0);

  const double total_duration{ 4.0 };
  plan_to_split.copy_onto_back(u0, 1.0);
  plan_to_split.copy_onto_back(u0 * 2, 1.0);
  plan_to_split.copy_onto_back(u0 * 3, 1.0);
  plan_to_split.copy_onto_back(u0 * 4, 1.0);

  const double duration_1p1{ 1.1 };
  prx::plan_t plan_1p1{ plan_to_split.split(duration_1p1) };

  const double epsilon{ 0.001 };

  PRX_DBG_VARS(total_duration - duration_1p1, plan_1p1.duration());
  BOOST_REQUIRE_CLOSE(total_duration - duration_1p1, plan_1p1.duration(), epsilon);
  BOOST_REQUIRE_CLOSE(duration_1p1, plan_to_split.duration(), epsilon);

  BOOST_REQUIRE_MESSAGE(u0[0] == plan_to_split[0].control->at(0), EXPECTED_GOT(u0[0], plan_to_split[0].control->at(0)));

  BOOST_REQUIRE_MESSAGE(u0[1] * 2 == plan_to_split[1].control->at(0),
                        EXPECTED_GOT(u0[1] * 2, plan_to_split[1].control->at(0)));

  BOOST_REQUIRE_MESSAGE(u0[1] * 2 == plan_1p1[0].control->at(0), EXPECTED_GOT(u0[1] * 2, plan_1p1[0].control->at(0)));

  BOOST_REQUIRE_MESSAGE(plan_1p1.size() == 3, EXPECTED_GOT(3, plan_1p1.size()));

  BOOST_REQUIRE_MESSAGE(plan_to_split.size() == 2, EXPECTED_GOT(2, plan_to_split.size()));
}