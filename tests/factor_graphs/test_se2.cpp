#define BOOST_AUTO_TEST_MAIN se3_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/defs.hpp"
#include "prx/factor_graphs/lie_groups/se2.hpp"

using SE2 = prx::fg::SE2_t;
using Angle = SE2::Angle;
using Rotation = SE2::Rotation;
using Translation = SE2::Translation;

BOOST_AUTO_TEST_CASE(constructor_test)
{
  const SE2 t0{};                                                    // default constructor
  const SE2 t1{ Translation::Zero(), 0.0 };                          // (vec, angle)
  const SE2 t2{ Translation::Zero(), Rotation::Identity() };         // (vec, Rotation)
  const SE2 t3{ Translation::Zero(), Eigen::Matrix2d::Identity() };  // (vec, Matrix2d)
  const SE2 t4{ 0, 0, 0 };                                           // (vec, Rotation)
  const SE2 t5{ gtsam::Pose2(0.0, 0.0, 0.0) };                       // (vec, Rotation)

  BOOST_REQUIRE_MESSAGE(t0.equals(t1), "t0 not equals t1");
  BOOST_REQUIRE_MESSAGE(t0.equals(t2), "t0 not equals t2");
  BOOST_REQUIRE_MESSAGE(t0.equals(t3), "t0 not equals t3");
  BOOST_REQUIRE_MESSAGE(t0.equals(t4), "t0 not equals t4");
  BOOST_REQUIRE_MESSAGE(t0.equals(t5), "t0 not equals t5");
}
