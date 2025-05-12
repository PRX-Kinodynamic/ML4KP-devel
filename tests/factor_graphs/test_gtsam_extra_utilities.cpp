#define BOOST_AUTO_TEST_MAIN se3_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/factor_graphs/utilities/gtsam_extra_utilities.hpp"

namespace mock
{
int foo(const double& x, const double& y0,                   // no-lint
        boost::optional<Eigen::MatrixXd&> Hx = boost::none,  // no-lint
        boost::optional<Eigen::MatrixXd&> Hy = boost::none)
{
  int res{ 0 };
  if (Hx)
    res = 1;
  if (Hy)
    res += 10;
  return res;
}
}  // namespace mock

BOOST_AUTO_TEST_CASE(check_optional_test)
{
  Eigen::MatrixXd mat;
  BOOST_REQUIRE(mock::foo(0, 1, prx::fg::check_optional(mat, false), prx::fg::check_optional(mat, false)) == 0);
  BOOST_REQUIRE(mock::foo(0, 1, prx::fg::check_optional(mat, true), prx::fg::check_optional(mat, false)) == 1);
  BOOST_REQUIRE(mock::foo(0, 1, prx::fg::check_optional(mat, false), prx::fg::check_optional(mat, true)) == 10);
  BOOST_REQUIRE(mock::foo(0, 1, prx::fg::check_optional(mat, true), prx::fg::check_optional(mat, true)) == 11);
  // BOOST_REQUIRE(prx::fg::check_optional());
  // BOOST_REQUIRE(prx::fg::check_optional());
  // BOOST_REQUIRE(prx::fg::check_optional());
  // BOOST_REQUIRE(prx::fg::check_optional());
}
