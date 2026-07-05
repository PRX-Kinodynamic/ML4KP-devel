#define BOOST_AUTO_TEST_MAIN lie_utils_test
#include <string>
#include <fstream>
#include <gtsam/geometry/Rot3.h>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/general/debug_utils.hpp"
#include "prx/utilities/math/lie_utils.hpp"

BOOST_AUTO_TEST_CASE(tangent_between_identity_test)
{
  gtsam::Rot3 x{ gtsam::Rot3::Identity() };
  gtsam::Rot3 y{ gtsam::Rot3::Identity() };

  const Eigen::Vector<double, 3> tgbtw{ prx::TangentBetween(x, y) };
  BOOST_REQUIRE(tgbtw.isZero());
}
