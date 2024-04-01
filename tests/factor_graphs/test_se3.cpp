#define BOOST_AUTO_TEST_MAIN se3_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/defs.hpp"
#include "prx/factor_graphs/lie_groups/se3.hpp"

using prx::fg::se3_t;
using Quaternion = Eigen::Quaterniond;
using Position = Eigen::Vector<double, 3>;

BOOST_AUTO_TEST_CASE(constructor_test)
{
  const se3_t se3(Quaternion::Identity(), Position::Zero());

  const Quaternion q_expected{ Quaternion::Identity() };
  const Position p_expected{ Position::Zero() };
  BOOST_REQUIRE_MESSAGE(q_expected.isApprox(se3.quaternion()), EXPECTED_GOT(q_expected, se3.quaternion()));
  BOOST_REQUIRE_MESSAGE(p_expected.isApprox(se3.position()), EXPECTED_GOT(p_expected, se3.position()));
}

BOOST_AUTO_TEST_CASE(composition_test)
{
  using RotationMatrix = Eigen::Matrix<double, 3, 3>;
  Position p0(0, -2, 0);
  Position p1(-1, 1, 0);
  Position p01(0, -3, -1);
  RotationMatrix R0{}, R1{}, R01{};
  R0 << 0, 0, 1, 0, -1, 0, 1, 0, 0;
  R1 << -1, 0, 0, 0, 0, 1, 0, 1, 0;
  R01 << 0, 1, 0, 0, 0, -1, -1, 0, 0;

  const se3_t se3_0(R0, p0);
  const se3_t se3_1(R1, p1);
  const se3_t se3_01{ se3_0 * se3_1 };

  BOOST_REQUIRE_MESSAGE(R01.isApprox(se3_01.matrix()), EXPECTED_GOT(R01, se3_01.matrix()));
  BOOST_REQUIRE_MESSAGE(p01.isApprox(se3_01.position()), EXPECTED_GOT(p01, se3_01.position()));
}