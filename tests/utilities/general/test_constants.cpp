#define BOOST_AUTO_TEST_MAIN constants_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/general/constants.hpp"

BOOST_AUTO_TEST_CASE(paths_test)
{
  BOOST_CHECK(!prx::lib_path.empty());
  BOOST_CHECK(!prx::mj_models_path.empty());
  BOOST_CHECK(!prx::input_path.empty());
  BOOST_CHECK(!prx::js_path.empty());
  BOOST_CHECK(!prx::out_path.empty());
}

BOOST_AUTO_TEST_CASE(are_approx_equal_decimals_test)
{
  BOOST_CHECK((prx::are_approx_equal<double, double>(M_PI, PRX_PI, PRX_EPSILON)));
  BOOST_CHECK((prx::are_approx_equal<double, float>(M_PI, PRX_PI, PRX_EPSILON)));
  BOOST_CHECK((prx::are_approx_equal<double, int>(M_PI, 3, 0.2)));
}

BOOST_AUTO_TEST_CASE(are_approx_equal_std_vector_test)
{
  std::vector<double> double_vec{ 0.00, 1.10, 2.20, 3.30, 4.40, 5.50 };
  std::vector<float> float_vec{ 0.01, 1.11, 2.21, 3.31, 4.41, 5.51 };
  std::vector<int> int_vec{ 0, 1, 2, 3, 4, 5 };
  BOOST_CHECK(prx::are_approx_equal(double_vec, float_vec, 0.02));
  BOOST_CHECK(prx::are_approx_equal(double_vec, int_vec, 0.51));
}

BOOST_AUTO_TEST_CASE(norm_angle_pi_test)
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

BOOST_AUTO_TEST_CASE(split_block_test)
{
  using Line = std::vector<double>;
  using Block = std::vector<Line>;
  using Columns = std::vector<std::size_t>;
  using ColumnsQuery = std::vector<Columns>;

  Block block0{};
  block0.emplace_back(Line{ { 00, 01, 02, 03 } });
  block0.emplace_back(Line{ { 10, 11, 12, 13 } });
  block0.emplace_back(Line{ { 20, 21, 22, 23 } });

  Columns c0{ 0 };
  Columns c1{ { 1, 2, 3 } };
  ColumnsQuery column_query{ c0, c1 };

  Block expected_0{};
  expected_0.emplace_back(Line{ { 01, 02, 03 } });
  expected_0.emplace_back(Line{ { 11, 12, 13 } });
  expected_0.emplace_back(Line{ { 21, 22, 23 } });

  Block expected_1{};
  expected_1.emplace_back(Line{ 00 });
  expected_1.emplace_back(Line{ 10 });
  expected_1.emplace_back(Line{ 20 });

  Block block1{ prx::split_block(block0, column_query) };

  const std::size_t num_lines{ 3 };
  for (int i = 0; i < num_lines; ++i)
  {
    BOOST_REQUIRE_EQUAL_COLLECTIONS(block0[i].begin(), block0[i].end(), expected_0[i].begin(), expected_0[i].end());
  }
  for (int i = 0; i < num_lines; ++i)
  {
    BOOST_REQUIRE_EQUAL_COLLECTIONS(block1[i].begin(), block1[i].end(), expected_1[i].begin(), expected_1[i].end());
  }
}