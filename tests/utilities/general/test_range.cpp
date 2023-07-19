#define BOOST_AUTO_TEST_MAIN range_test
#include <string>
#include <boost/test/unit_test.hpp>

#include "prx/utilities/general/range.hpp"
#include "prx/utilities/general/zipped_iter.hpp"

BOOST_AUTO_TEST_CASE(index_range_test)
{
  int i{ 0 };
  const int expected_i{ 100 };
  for (auto e : prx::index_range_t())
  {
    BOOST_CHECK(e == i);
    i++;
    if (i == 100)
      break;
  }
  // Checking that the for was fully executed
  BOOST_CHECK(i == expected_i);
}

BOOST_AUTO_TEST_CASE(index_range_custom_start_test)
{
  int i{ 0 };
  const int expected_i{ 100 };
  for (auto e : prx::index_range_t(-100))
  {
    BOOST_CHECK(e + 100 == i);
    i++;
    if (i == expected_i)
      break;
  }
  // Checking that the for was fully executed
  BOOST_CHECK(i == expected_i);
}

BOOST_AUTO_TEST_CASE(index_range_custom_start_end_test)
{
  int i{ 0 };
  const int expected_i{ 50 };
  for (auto e : prx::index_range_t(-100, -50))
  {
    BOOST_CHECK(e + 100 == i);
    i++;
    if (i == expected_i)
      break;
  }
  // Checking that the for was fully executed
  BOOST_CHECK(i == expected_i);
}

BOOST_AUTO_TEST_CASE(index_range_custom_start_end_increment_test)
{
  int i{ 0 };
  const int expected_i{ 200 };
  for (auto e : prx::index_range_t(-100, 100, 2))
  {
    BOOST_CHECK(e + 100 == i);
    i += 2;
    if (i == expected_i)
      break;
  }
  // Checking that the for was fully executed
  BOOST_CHECK(i == expected_i);
}

BOOST_AUTO_TEST_CASE(double_range_custom_start_end_increment_test)
{
  double i{ 0 };
  const double expected_i{ 1.0 };
  for (auto e : prx::range_t<double>(0, 1, 0.01))
  {
    BOOST_CHECK_MESSAGE(e == i, "Expected: " << i << " got: " << e);
    i += 0.01;
    if (i > expected_i)
      break;
  }
  // Checking that the for was fully executed
  BOOST_CHECK_MESSAGE(expected_i < i, "Expected: " << expected_i << " got: " << i);
}
