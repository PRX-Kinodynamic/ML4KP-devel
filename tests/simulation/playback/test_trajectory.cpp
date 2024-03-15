#define BOOST_AUTO_TEST_MAIN constants_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/simulation/playback/trajectory.hpp"

namespace prx
{
double simulation_step;
};

namespace mock
{
struct trajectory_space_test_t
{
  trajectory_space_test_t() : x0(0), x1(0), address({ &x0, &x1 }), space("EE", address, "space_test")
  {
    prx::simulation_step = 0.1;
  }
  double x0, x1;
  std::vector<double*> address;
  prx::space_t space;
};
}  // namespace mock

BOOST_AUTO_TEST_CASE(trajectory_construction_test)
{
  mock::trajectory_space_test_t test;
  prx::space_t& space(test.space);
  prx::trajectory_t traj(&space);

  BOOST_REQUIRE(traj.size() == 0);
}

BOOST_AUTO_TEST_CASE(trajectory_resize_test)
{
  mock::trajectory_space_test_t test;
  prx::space_t& space(test.space);
  prx::trajectory_t traj(&space);

  BOOST_REQUIRE(traj.size() == 0);

  traj.resize(5);

  BOOST_REQUIRE(traj.size() == 5);
}

BOOST_AUTO_TEST_CASE(trajectory_duration_test)
{
  mock::trajectory_space_test_t test;
  prx::space_t& space(test.space);
  prx::trajectory_t traj(&space);

  for (int i = 0; i < 51; ++i)
  {
    traj.push_back(Eigen::Vector2d(0, i));
  }
  const double expected_duration{ 5 };
  BOOST_REQUIRE_MESSAGE(traj.duration() == expected_duration, EXPECTED_GOT(expected_duration, traj.duration()));
}

BOOST_AUTO_TEST_CASE(trajectory_at_unormalized_test)
{
  mock::trajectory_space_test_t test;
  prx::space_t& space(test.space);
  prx::trajectory_t traj(&space);

  const double expected_duration{ 2 };
  double ti = 0;
  for (; ti <= expected_duration; ti += prx::simulation_step)
  {
    traj.push_back(Eigen::Vector2d(ti, ti));
  }
  traj.push_back(Eigen::Vector2d(ti, ti));

  const prx::space_point_t expected_0p0{ space.make_point(Eigen::Vector2d::Zero()) };
  const prx::space_point_t expected_0p5{ space.make_point(Eigen::Vector2d(0.5, 0.5)) };
  const prx::space_point_t expected_1p0{ space.make_point(Eigen::Vector2d(1.0, 1.0)) };
  const prx::space_point_t expected_1p5{ space.make_point(Eigen::Vector2d(1.5, 1.5)) };
  const prx::space_point_t expected_2p0{ space.make_point(Eigen::Vector2d(2.0, 2.0)) };

  const prx::space_point_t result_0p0{ traj.at(0.0, false) };
  const prx::space_point_t result_0p5{ traj.at(0.5, false) };
  const prx::space_point_t result_1p0{ traj.at(1.0, false) };
  const prx::space_point_t result_1p5{ traj.at(1.5, false) };
  const prx::space_point_t result_2p0{ traj.at(2.0, false) };

  BOOST_REQUIRE_MESSAGE(space.equal_points(expected_0p0, result_0p0), EXPECTED_GOT(expected_0p0, result_0p0));
  BOOST_REQUIRE_MESSAGE(space.equal_points(expected_0p5, result_0p5), EXPECTED_GOT(expected_0p5, result_0p5));
  BOOST_REQUIRE_MESSAGE(space.equal_points(expected_1p0, result_1p0), EXPECTED_GOT(expected_1p0, result_1p0));
  BOOST_REQUIRE_MESSAGE(space.equal_points(expected_1p5, result_1p5), EXPECTED_GOT(expected_1p5, result_1p5));
  BOOST_REQUIRE_MESSAGE(space.equal_points(expected_2p0, result_2p0), EXPECTED_GOT(expected_2p0, result_2p0));
}

BOOST_AUTO_TEST_CASE(trajectory_index_at_time)
{
  mock::trajectory_space_test_t test;
  prx::space_t& space(test.space);
  prx::trajectory_t traj(&space);

  traj.push_back(Eigen::Vector2d(0.0, 0.0));  // 0.0
  traj.push_back(Eigen::Vector2d(1.0, 1.0));  // 0.1
  traj.push_back(Eigen::Vector2d(2.0, 2.0));  // 0.2

  // prx::simulation_step is 0.1;
  const double query_t0{ 0.0 };                               // 0.0
  const double query_t1{ prx::simulation_step / 3.0 };        // t=0.3 between (0,0) and (1,1)
  const double query_t2{ prx::simulation_step / 2.0 };        // t=0.5 between (0,0) and (1,1)
  const double query_t3{ 3.0 * prx::simulation_step / 4.0 };  // t=0.75 between (0,0) and (1,1)
  const double query_t4{ prx::simulation_step };              // t=0.25 between (1,1) and (2,2)
  const double query_t5{ 5.0 * prx::simulation_step / 4.0 };  // t=0.25 between (1,1) and (2,2)

  const std::size_t res_idx0{ traj.index_at_time(query_t0) };
  const std::size_t res_idx1{ traj.index_at_time(query_t1) };
  const std::size_t res_idx2{ traj.index_at_time(query_t2) };
  const std::size_t res_idx3{ traj.index_at_time(query_t3) };
  const std::size_t res_idx4{ traj.index_at_time(query_t4) };
  const std::size_t res_idx5{ traj.index_at_time(query_t5) };

  const std::size_t expected_0123{ 0 };
  const std::size_t expected_45{ 1 };

  BOOST_REQUIRE_MESSAGE(expected_0123 == res_idx0, EXPECTED_GOT(expected_0123, res_idx0));
  BOOST_REQUIRE_MESSAGE(expected_0123 == res_idx1, EXPECTED_GOT(expected_0123, res_idx1));
  BOOST_REQUIRE_MESSAGE(expected_0123 == res_idx2, EXPECTED_GOT(expected_0123, res_idx2));
  BOOST_REQUIRE_MESSAGE(expected_0123 == res_idx3, EXPECTED_GOT(expected_0123, res_idx3));
  BOOST_REQUIRE_MESSAGE(expected_45 == res_idx4, EXPECTED_GOT(expected_45, res_idx4));
  BOOST_REQUIRE_MESSAGE(expected_45 == res_idx5, EXPECTED_GOT(expected_45, res_idx5));
}