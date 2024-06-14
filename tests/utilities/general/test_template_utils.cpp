#define BOOST_AUTO_TEST_MAIN template_utils_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/template_utils.hpp"

BOOST_AUTO_TEST_CASE(shared_ptr_test)
{
  BOOST_REQUIRE(prx::utilities::is_shared_ptr<std::shared_ptr<double>>::value);
  BOOST_REQUIRE(prx::utilities::is_shared_ptr<const std::shared_ptr<double>>::value);

  BOOST_REQUIRE(not prx::utilities::is_shared_ptr<double>::value);
  BOOST_REQUIRE(not prx::utilities::is_shared_ptr<double*>::value);
  BOOST_REQUIRE(not prx::utilities::is_shared_ptr<std::unique_ptr<double>>::value);
}

BOOST_AUTO_TEST_CASE(unique_ptr_test)
{
  BOOST_REQUIRE(prx::utilities::is_unique_ptr<std::unique_ptr<double>>::value);
  BOOST_REQUIRE(prx::utilities::is_unique_ptr<const std::unique_ptr<double>>::value);

  BOOST_REQUIRE(not prx::utilities::is_unique_ptr<double>::value);
  BOOST_REQUIRE(not prx::utilities::is_unique_ptr<double*>::value);
  BOOST_REQUIRE(not prx::utilities::is_unique_ptr<std::shared_ptr<double>>::value);
}

BOOST_AUTO_TEST_CASE(any_ptr_test)
{
  BOOST_REQUIRE(prx::utilities::is_any_ptr<std::shared_ptr<double>>::value);
  BOOST_REQUIRE(prx::utilities::is_any_ptr<std::unique_ptr<double>>::value);
  BOOST_REQUIRE(prx::utilities::is_any_ptr<double*>::value);

  BOOST_REQUIRE(not prx::utilities::is_any_ptr<double>::value);
}

BOOST_AUTO_TEST_CASE(is_streamable_test)
{
  BOOST_REQUIRE(prx::utilities::is_streamable<std::string>::value);
  BOOST_REQUIRE(prx::utilities::is_streamable<double>::value);
  BOOST_REQUIRE(not prx::utilities::is_streamable<std::vector<double>>::value);
}
