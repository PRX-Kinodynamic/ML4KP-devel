#define BOOST_AUTO_TEST_MAIN constants_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/general/constants.hpp"

BOOST_AUTO_TEST_CASE(paths_test)
{
  BOOST_CHECK(!prx::lib_path.empty());
  BOOST_CHECK(!prx::models_path.empty());
  BOOST_CHECK(!prx::input_path.empty());
  BOOST_CHECK(!prx::js_path.empty());
  BOOST_CHECK(!prx::out_path.empty());
}

BOOST_AUTO_TEST_CASE(are_approx_equal_decimals_test)
{
  BOOST_CHECK((prx::are_approx_equal<double>(M_PI, PRX_PI, PRX_EPSILON)));
  BOOST_CHECK((prx::are_approx_equal<float>(M_PI, PRX_PI, PRX_EPSILON)));
  BOOST_CHECK((prx::are_approx_equal<double>(M_PI, 3, 0.2)));
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
