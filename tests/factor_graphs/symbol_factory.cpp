#define BOOST_AUTO_TEST_MAIN symbols_factory_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/factor_graphs/defs.hpp"

BOOST_AUTO_TEST_CASE(symbols_factory_test)
{
  std::vector<std::string> symbol_names = { "goal_symbol", "state_symbol", "control_symbol", "time_symbol" };

  // std::vector<std::pair<std::string, std::string>> fn_names =
  //     {{"2D_Point", "2D_Point"}, {"rally_car", "rally_car"}, {"treaded_vehicle", "treaded_vehicle"},
  //     {"FO_treaded_vehicle", "FO_treaded_vehicle"}};

  auto available_symbols = prx::symbol_factory_t::available_symbols();
  for (auto name : symbol_names)
  {
    printf("\tChecking available system: %s...", name.c_str());
    BOOST_CHECK(std::find(available_symbols.begin(), available_symbols.end(), name) != available_symbols.end());
    printf("\t[ OK ]\n");
  }

  printf("Checking symbols in Factory...\n");
  {
    printf("\tChecking symbol: %s...", symbol_names[0].c_str());
    auto symbol = prx::symbol_factory_t::create_symbol(symbol_names[0], 1);
    // BOOST_CHECK( sys_ptr != nullptr );
    BOOST_CHECK(symbol.label() == "Xg");
    printf("\t[ OK ]\n");
  }

  {
    printf("\tChecking symbol: %s...", symbol_names[1].c_str());
    uint64_t ti = 10;
    auto symbol = prx::symbol_factory_t::create_symbol(symbol_names[1], ti);
    BOOST_CHECK(symbol.label() == "Xi");
    BOOST_CHECK(symbol.time() == ti);
    printf("\t[ OK ]\n");
  }

  {
    printf("\tChecking symbol: %s...", symbol_names[2].c_str());
    uint64_t ti = 100;
    auto symbol = prx::symbol_factory_t::create_symbol(symbol_names[2], ti);
    BOOST_CHECK(symbol.label() == "Ui");
    BOOST_CHECK(symbol.time() == ti);
    printf("\t[ OK ]\n");
  }

  {
    printf("\tChecking symbol: %s...", symbol_names[3].c_str());
    uint64_t ti = 50;
    auto symbol = prx::symbol_factory_t::create_symbol(symbol_names[3], ti);
    BOOST_CHECK(symbol.label() == "ti");
    BOOST_CHECK(symbol.time() == ti);
    printf("\t[ OK ]\n");
  }
}

BOOST_AUTO_TEST_CASE(hashed_symbols_creation_test)
{
  const gtsam::Key s0{ prx::symbol_factory_t::create_hashed_symbol("test", 0) };
  const gtsam::Key s1{ prx::symbol_factory_t::create_hashed_symbol("test", 1) };
  const gtsam::Key s2{ prx::symbol_factory_t::create_hashed_symbol("test", 2) };
  const gtsam::Key s3{ prx::symbol_factory_t::create_hashed_symbol("test", 3) };

  const std::string expected_0{ "test0" };
  const std::string expected_1{ "test1" };
  const std::string expected_2{ "test2" };
  const std::string expected_3{ "test3" };

  std::string result_0{ prx::symbol_factory_t::formatter(s0) };
  std::string result_1{ prx::symbol_factory_t::formatter(s1) };
  std::string result_2{ prx::symbol_factory_t::formatter(s2) };
  std::string result_3{ prx::symbol_factory_t::formatter(s3) };

  BOOST_CHECK_MESSAGE(result_0 == expected_0, "Got: " << result_0 << " expected: " << expected_0);
  BOOST_CHECK_MESSAGE(result_1 == expected_1, "Got: " << result_1 << " expected: " << expected_1);
  BOOST_CHECK_MESSAGE(result_2 == expected_2, "Got: " << result_2 << " expected: " << expected_2);
  BOOST_CHECK_MESSAGE(result_3 == expected_3, "Got: " << result_3 << " expected: " << expected_3);
}

BOOST_AUTO_TEST_CASE(repeated_hashed_symbols_creation_test)
{
  const gtsam::Key s0_0{ prx::symbol_factory_t::create_hashed_symbol("test", 0) };
  const gtsam::Key s0_1{ prx::symbol_factory_t::create_hashed_symbol("test", 0) };

  std::string expected_0_0{ "test0" };
  std::string expected_0_1{ "test0" };

  std::string result_0_0{ prx::symbol_factory_t::formatter(s0_0) };
  std::string result_0_1{ prx::symbol_factory_t::formatter(s0_1) };

  BOOST_CHECK_MESSAGE(s0_0 == s0_1, "Expected symbol " << s0_0 << " to be equal to " << s0_1);
  BOOST_CHECK_MESSAGE(result_0_0 == expected_0_0, "Got: " << result_0_0 << " expected: " << expected_0_0);
  BOOST_CHECK_MESSAGE(result_0_1 == expected_0_1, "Got: " << result_0_1 << " expected: " << expected_0_1);
}