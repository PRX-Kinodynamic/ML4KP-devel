#define BOOST_AUTO_TEST_MAIN optional_utils_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/general/optional_utils.hpp"
#include <boost/optional.hpp>

BOOST_AUTO_TEST_CASE(any_of_test)
{
  boost::optional<int> x0{ 0 };
  boost::optional<int> x1{ 1 };
  boost::optional<int> x2{ boost::none };
  boost::optional<int> x3{ boost::none };

  BOOST_CHECK(prx::utilities::any_of(x0));
  BOOST_CHECK(prx::utilities::any_of(x0, x1));
  BOOST_CHECK(prx::utilities::any_of(x0, x1, x2));
  BOOST_CHECK(not prx::utilities::any_of(x2));
  BOOST_CHECK(not prx::utilities::any_of(x2, x3));
  BOOST_CHECK(prx::utilities::any_of(x2, x3, x0));
}