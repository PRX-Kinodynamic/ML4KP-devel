#define BOOST_AUTO_TEST_MAIN quaternion_test
#include <boost/test/unit_test.hpp>
#include "prx/utilities/defs.hpp"

using namespace prx;
BOOST_AUTO_TEST_CASE(quaternion_to_euler_zero)
{
    quaternion_t quat = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    auto euler = quaternion_to_euler(quat);

    BOOST_CHECK(euler[0] == 0.0);
    BOOST_CHECK(euler[1] == 0.0);
    BOOST_CHECK(euler[2] == 0.0);
}