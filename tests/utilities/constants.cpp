#define BOOST_AUTO_TEST_MAIN constants_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/general/constants.hpp"


BOOST_AUTO_TEST_CASE( constants_test )
{   
	double a1 = M_PI;
	double lower = 0;
	double upper = 2 * M_PI;

    BOOST_CHECK(prx::norm_angle_pi(a1, lower, upper) == a1);
    a1 = a1 + upper;
    BOOST_CHECK(prx::norm_angle_pi(a1, lower, upper) == M_PI);
    a1 = lower;
    BOOST_CHECK(prx::norm_angle_pi(a1, lower, upper) == a1);
    a1 = upper;
    BOOST_CHECK(prx::norm_angle_pi(a1, lower, upper) == a1);
    

}
