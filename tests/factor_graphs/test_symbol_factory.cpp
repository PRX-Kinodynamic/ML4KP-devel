#define BOOST_AUTO_TEST_MAIN symbols_factory_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/factor_graphs/utilities/symbols_factory.hpp"

using prx::fg::symbol_factory_t;
BOOST_AUTO_TEST_CASE(hashed_symbols_creation_test)
{
  const gtsam::Key s0{ symbol_factory_t::create_hashed_symbol("test", 0) };
  const gtsam::Key s1{ symbol_factory_t::create_hashed_symbol("test", 1) };
  const gtsam::Key s2{ symbol_factory_t::create_hashed_symbol("test", 2) };
  const gtsam::Key s3{ symbol_factory_t::create_hashed_symbol("test", 3) };

  const std::string expected_0{ "test0" };
  const std::string expected_1{ "test1" };
  const std::string expected_2{ "test2" };
  const std::string expected_3{ "test3" };

  std::string result_0{ symbol_factory_t::formatter(s0) };
  std::string result_1{ symbol_factory_t::formatter(s1) };
  std::string result_2{ symbol_factory_t::formatter(s2) };
  std::string result_3{ symbol_factory_t::formatter(s3) };

  BOOST_CHECK_MESSAGE(result_0 == expected_0, "Got: " << result_0 << " expected: " << expected_0);
  BOOST_CHECK_MESSAGE(result_1 == expected_1, "Got: " << result_1 << " expected: " << expected_1);
  BOOST_CHECK_MESSAGE(result_2 == expected_2, "Got: " << result_2 << " expected: " << expected_2);
  BOOST_CHECK_MESSAGE(result_3 == expected_3, "Got: " << result_3 << " expected: " << expected_3);
}

BOOST_AUTO_TEST_CASE(repeated_hashed_symbols_creation_test)
{
  const gtsam::Key s0_0{ symbol_factory_t::create_hashed_symbol("test", 0) };
  const gtsam::Key s0_1{ symbol_factory_t::create_hashed_symbol("test", 0) };

  std::string expected_0_0{ "test0" };
  std::string expected_0_1{ "test0" };

  std::string result_0_0{ symbol_factory_t::formatter(s0_0) };
  std::string result_0_1{ symbol_factory_t::formatter(s0_1) };

  BOOST_CHECK_MESSAGE(s0_0 == s0_1, "Expected symbol " << s0_0 << " to be equal to " << s0_1);
  BOOST_CHECK_MESSAGE(result_0_0 == expected_0_0, "Got: " << result_0_0 << " expected: " << expected_0_0);
  BOOST_CHECK_MESSAGE(result_0_1 == expected_0_1, "Got: " << result_0_1 << " expected: " << expected_0_1);
}