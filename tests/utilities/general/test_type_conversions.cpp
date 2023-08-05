#define BOOST_AUTO_TEST_MAIN type_conversions_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/type_conversions.hpp"

using prx::utilities::convert_to;
BOOST_AUTO_TEST_CASE(arithmetic_to_string)
{
  prx::constants::precision = 2;
  const int int_test{ 3 };
  const double dbl_test{ 3.14 };

  const std::string expected_int{ "3" };
  const std::string expected_dbl{ "3.14" };

  const std::string res_int{ convert_to<std::string>(int_test) };
  const std::string res_dbl{ convert_to<std::string>(dbl_test) };

  BOOST_CHECK_MESSAGE(expected_int == res_int, EXPECTED_GOT(expected_int, res_int));
  BOOST_CHECK_MESSAGE(expected_dbl == res_dbl, EXPECTED_GOT(expected_dbl, res_dbl));
}

BOOST_AUTO_TEST_CASE(string_converts_to_integral)
{
  const std::string str_int{ "0" };
  const std::string str_long{ "1" };
  const std::string str_long_long{ "2" };
  const std::string str_size_t{ "3" };

  const int result_int{ convert_to<int>(str_int) };
  const long result_long{ convert_to<long>(str_long) };
  const long long result_long_long{ convert_to<long long>(str_long_long) };
  const std::size_t result_size_t{ convert_to<std::size_t>(str_size_t) };

  const int expected_int{ 0 };
  const long expected_long{ 1 };
  const long long expected_long_long{ 2 };
  const std::size_t expected_size_t{ 3 };

  BOOST_CHECK_MESSAGE(expected_int == result_int, EXPECTED_GOT(expected_int, result_int));
  BOOST_CHECK_MESSAGE(expected_long == result_long, EXPECTED_GOT(expected_long, result_long));
  BOOST_CHECK_MESSAGE(expected_long_long == result_long_long, EXPECTED_GOT(expected_long_long, result_long_long));
  BOOST_CHECK_MESSAGE(expected_size_t == result_size_t, EXPECTED_GOT(expected_size_t, result_size_t));
}

BOOST_AUTO_TEST_CASE(string_converts_to_double)
{
  const std::string str_double{ "3.14159" };

  const double result_double{ convert_to<double>(str_double) };

  const double expected_double{ 3.14159 };

  BOOST_CHECK_MESSAGE(expected_double == result_double, EXPECTED_GOT(expected_double, result_double));
}

// Passthrough: From T to T
BOOST_AUTO_TEST_CASE(passthrough_convertions)
{
  const std::string _str{ "abc" };
  const int _int{ 1 };
  const double _dbl{ 3.1415 };

  const std::string str_res{ convert_to<std::string>(_str) };
  const int int_res{ convert_to<int>(_int) };
  const double dbl_res{ convert_to<double>(_dbl) };

  BOOST_CHECK_MESSAGE(_str == str_res, EXPECTED_GOT(_str, str_res));
  BOOST_CHECK_MESSAGE(_int == int_res, EXPECTED_GOT(_int, int_res));
  BOOST_CHECK_MESSAGE(_dbl == dbl_res, EXPECTED_GOT(_dbl, dbl_res));
}