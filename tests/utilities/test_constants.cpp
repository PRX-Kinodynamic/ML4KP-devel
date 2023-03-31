#define BOOST_AUTO_TEST_MAIN constants_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/general/constants.hpp"

BOOST_AUTO_TEST_CASE(constants_test)
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

BOOST_AUTO_TEST_CASE(split_string_test)
{
  std::string str{ "a bb  ccc      dddd" };
  std::vector<std::string> expected = { "a", "bb", "ccc", "dddd" };
  std::vector<std::string> result = prx::split<std::string>(str, ' ');
  BOOST_CHECK_MESSAGE(result.size() == expected.size(),
                      "Wrong size, got: '" << result.size() << "' but '" << expected.size() << "' was expected.");
  for (int i = 0; i < expected.size(); ++i)
  {
    BOOST_CHECK_MESSAGE(result[i] == expected[i],
                        "Wrong element, got: '" << result[i] << "' but '" << expected[i] << "' was expected.");
  }
}

BOOST_AUTO_TEST_CASE(split_double_test)
{
  std::string line{ "1 2.0  3.1415      4.5" };
  std::vector<double> expected = { 1, 2.0, 3.1415, 4.5 };
  std::vector<double> result = prx::split<double>(line, ' ');
  BOOST_CHECK_MESSAGE(result.size() == expected.size(),
                      "Wrong size, got: '" << result.size() << "' but '" << expected.size() << "' was expected.");
  for (int i = 0; i < expected.size(); ++i)
  {
    BOOST_CHECK_MESSAGE(result[i] == expected[i],
                        "Wrong element, got: '" << result[i] << "' but '" << expected[i] << "' was expected.");
  }
}