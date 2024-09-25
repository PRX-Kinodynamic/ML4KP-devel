#define BOOST_AUTO_TEST_MAIN quaternion_test
#include <boost/test/unit_test.hpp>
#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/transforms.hpp"

using namespace prx;
using Quaternion = Eigen::Quaterniond;
bool check_quaternions(const Quaternion& q0, const Quaternion& q1, const double epsilon = 0.01)
{
  const bool w_eq{ std::fabs(q0.w() - q1.w()) < epsilon };
  const bool x_eq{ std::fabs(q0.x() - q1.x()) < epsilon };
  const bool y_eq{ std::fabs(q0.y() - q1.y()) < epsilon };
  const bool z_eq{ std::fabs(q0.z() - q1.z()) < epsilon };

  return w_eq and x_eq and y_eq and z_eq;
}

BOOST_AUTO_TEST_CASE(quaternion_to_euler_zero)
{
  quaternion_t quat = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
  auto euler = quaternion_to_euler(quat);

  BOOST_CHECK(euler[0] == 0.0);
  BOOST_CHECK(euler[1] == 0.0);
  BOOST_CHECK(euler[2] == 0.0);
}

BOOST_AUTO_TEST_CASE(quaternion_from_z_angle)
{
  const Eigen::Vector<double, 1> theta{ 0.0 };
  const Quaternion q0{ euler_to_rotation<Quaternion>(theta, "Z") };
  const Quaternion q0e{ Quaternion::Identity() };

  const Eigen::Vector<double, 1> theta1{ 1.5707963268 };
  const Quaternion q1{ euler_to_rotation<Quaternion>(theta1, "Z") };
  const Quaternion q1e{ 0.7071068, 0.0, 0.0, 0.7071068 };

  BOOST_CHECK_MESSAGE(check_quaternions(q0e, q0), EXPECTED_GOT(q0e, q0));
  BOOST_CHECK_MESSAGE(check_quaternions(q1e, q1), EXPECTED_GOT(q1e, q1));
}
