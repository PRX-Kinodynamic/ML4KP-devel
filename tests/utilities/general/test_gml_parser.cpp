#define BOOST_AUTO_TEST_MAIN gml_parser_test
#include <string>
#include <boost/test/unit_test.hpp>

#include "prx/utilities/general/gml_parser.hpp"

using prx::utilities::gml_generic_value_t;
using prx::utilities::gml_key_t;
using prx::utilities::gml_list_t;
using prx::utilities::gml_value_t;

BOOST_AUTO_TEST_CASE(testing_gml_key_constructor)
{
  const std::string valid_key_0{ "A" };
  const std::string valid_key_1{ "A0" };
  const std::string valid_key_2{ "A0A" };
  gml_key_t key_0(valid_key_0);
  gml_key_t key_1(valid_key_1);
  gml_key_t key_2(valid_key_2);
  BOOST_CHECK(std::string(key_0) == valid_key_0);
  BOOST_CHECK(std::string(key_1) == valid_key_1);
  BOOST_CHECK(std::string(key_2) == valid_key_2);

  // Expecting an assert to happen. Boost still prints to terminal so no worry about message
  BOOST_CHECK_THROW(gml_key_t("0A"), prx::prx_assert_t);
}

BOOST_AUTO_TEST_CASE(testing_gml_generic_value_constructor)
{
  const int int_value{ 10 };
  const double dbl_value{ 3.14 };
  const std::string str_value{ "Val" };

  const std::string expected_int_value{ "10" };
  const std::string expected_dbl_value{ "3.14" };
  const std::string expected_str_value{ "\"Val\"" };

  const gml_generic_value_t<int> int_gml_value(int_value);
  const gml_generic_value_t<double> dbl_gml_value(dbl_value);
  const gml_generic_value_t<std::string> str_gml_value(str_value);

  BOOST_CHECK(std::string(int_gml_value) == expected_int_value);
  BOOST_CHECK(std::string(dbl_gml_value) == expected_dbl_value);
  BOOST_CHECK(std::string(str_gml_value) == expected_str_value);
}

BOOST_AUTO_TEST_CASE(testing_gml_list_constructor)
{
  gml_list_t gml_list;

  gml_key_t key_0("int");
  gml_key_t key_1("dbl");
  gml_key_t key_2("str");

  const int int_gml_value(10);
  const double dbl_gml_value(3.14);
  const std::string str_gml_value("value");
  gml_list.emplace(key_0, int_gml_value);
  gml_list.emplace(key_1, dbl_gml_value);
  gml_list.emplace(key_2, str_gml_value);

  std::string expected_str("int 10\ndbl 3.14\nstr \"value\"\n");

  std::stringstream ss;
  ss << gml_list;
  const std::string gml_str{ ss.str() };
  BOOST_CHECK_MESSAGE(expected_str == gml_str, EXPECTED_GOT(expected_str, gml_str));
}

BOOST_AUTO_TEST_CASE(testing_gml_generic_value_with_list)
{
  gml_list_t gml_list;

  gml_key_t key_0("int");
  gml_key_t key_1("dbl");
  gml_key_t key_2("str");

  const int int_gml_value(10);
  const double dbl_gml_value(3.14);
  const std::string str_gml_value("value");
  gml_list.emplace(key_0, int_gml_value);
  gml_list.emplace(key_1, dbl_gml_value);
  gml_list.emplace(key_2, str_gml_value);

  std::string expected_str(" [\nint 10\ndbl 3.14\nstr \"value\"\n]\n");

  gml_generic_value_t<gml_list_t> value_list(gml_list);
  const std::string gml_str{ value_list };
  BOOST_CHECK_MESSAGE(expected_str == gml_str, EXPECTED_GOT(expected_str, gml_str));
}
