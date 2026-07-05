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

BOOST_AUTO_TEST_CASE(container_to_string_convertion)
{
  const std::vector<int> vec_ints{ { 0, 1, 2, 3 } };
  prx::constants::separating_value = ' ';
  const std::string expected_ints{ "0 1 2 3 " };

  const std::string result_ints{ convert_to<std::string>(vec_ints) };
  BOOST_CHECK_MESSAGE(expected_ints == result_ints, EXPECTED_GOT(expected_ints, result_ints));
}

BOOST_AUTO_TEST_CASE(auto_cast_test)
{
  const int int_const{ 1 };
  const double double_const{ 2.0 };
  const std::string str_const{ "3" };

  int int_var{ 0 };
  double double_var{ 0.0 };
  std::string str_var{ "0" };

  prx::utilities::auto_cast(int_var, int_const);
  BOOST_REQUIRE(int_var == 1);
  prx::utilities::auto_cast(int_var, double_const);
  BOOST_REQUIRE(int_var == 2);
  prx::utilities::auto_cast(int_var, str_const);
  BOOST_REQUIRE(int_var == 3);

  prx::utilities::auto_cast(double_var, int_const);
  BOOST_REQUIRE(double_var == 1);
  prx::utilities::auto_cast(double_var, double_const);
  BOOST_REQUIRE(double_var == 2);
  prx::utilities::auto_cast(double_var, str_const);
  BOOST_REQUIRE(double_var == 3);

  prx::utilities::auto_cast(str_var, int_const);
  BOOST_REQUIRE(str_var == "1");

  // Dbl to string is weird since it could be "2" or "2." or "2.0" or "2.00000..."
  // So at least to test, convert it back to a double
  prx::utilities::auto_cast(str_var, double_const);
  prx::utilities::auto_cast(double_var, str_var);
  BOOST_REQUIRE(double_var == 2.0);
  prx::utilities::auto_cast(str_var, str_const);
  BOOST_REQUIRE(str_var == "3");
}

BOOST_AUTO_TEST_CASE(string_hex_to_double_convertion)
{
  const char hex{ 'F' };
  const int hex_dbl{ convert_to<int>(hex) };

  PRX_DBG_VARS(hex_dbl)
  BOOST_REQUIRE(hex_dbl == 15);
}