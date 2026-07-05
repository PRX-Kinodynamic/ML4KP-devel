#define BOOST_AUTO_TEST_MAIN chi_squared_test
#include <string>
#include <fstream>
#include <gtsam/geometry/Rot2.h>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/general/debug_utils.hpp"
#include "prx/utilities/math/chi_squared.hpp"

BOOST_AUTO_TEST_CASE(chi_squared_1dof_test)
{
  prx::chi_squared chi2(0.0000001);

  const double cv_1_0p1{ chi2.critical_value(1, 0.1) };
  const double cv_1_0p1_2{ chi2.critical_value(1, 0.1) };
  const double cv_1_0p05{ chi2.critical_value(1, 0.05) };
  const double cv_1_0p005{ chi2.critical_value(1, 0.005) };

  BOOST_CHECK_SMALL(std::fabs(cv_1_0p1 - 2.706), 0.01);
  BOOST_CHECK_SMALL(std::fabs(cv_1_0p1_2 - 2.706), 0.01);
  BOOST_CHECK_SMALL(std::fabs(cv_1_0p05 - 3.841), 0.01);
  BOOST_CHECK_SMALL(std::fabs(cv_1_0p005 - 7.879), 0.01);
}

BOOST_AUTO_TEST_CASE(chi_squared_2dof_test)
{
  prx::chi_squared chi2(0.0000001);

  const double cv_2_0p1{ chi2.critical_value(2, 0.1) };
  const double cv_2_0p05{ chi2.critical_value(2, 0.05) };

  BOOST_CHECK_SMALL(std::fabs(cv_2_0p1 - 4.605), 0.01);
  BOOST_CHECK_SMALL(std::fabs(cv_2_0p05 - 5.991), 0.01);
}

BOOST_AUTO_TEST_CASE(chi_squared_50dof_test)
{
  prx::chi_squared chi2(0.0000001);

  const auto start0{ std::chrono::steady_clock::now() };
  const double cv_50_0p1{ chi2.critical_value(50, 0.1) };
  const auto finish0{ std::chrono::steady_clock::now() };
  const std::chrono::duration<double> elapsed_seconds0{ finish0 - start0 };
  auto secs0 = elapsed_seconds0.count();
  PRX_DBG_VARS(secs0)

  const auto start{ std::chrono::steady_clock::now() };
  const double cv_50_0p1p{ chi2.critical_value(50, 0.1) };
  const auto finish{ std::chrono::steady_clock::now() };
  const std::chrono::duration<double> elapsed_seconds{ finish - start };
  auto secs1 = elapsed_seconds.count();
  PRX_DBG_VARS(secs1)

  const auto start2{ std::chrono::steady_clock::now() };
  const double cv_50_0p05{ chi2.critical_value(50, 0.05) };
  const auto finish2{ std::chrono::steady_clock::now() };
  const std::chrono::duration<double> elapsed_seconds2{ finish2 - start2 };
  auto secs2 = elapsed_seconds2.count();
  PRX_DBG_VARS(secs2)

  BOOST_CHECK_SMALL(std::fabs(cv_50_0p1 - 63.167), 0.01);
  BOOST_CHECK_SMALL(std::fabs(cv_50_0p1p - 63.167), 0.01);
  BOOST_CHECK_SMALL(std::fabs(cv_50_0p05 - 67.505), 0.01);
}
