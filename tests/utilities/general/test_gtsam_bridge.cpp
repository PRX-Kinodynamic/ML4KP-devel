#include <gtsam/geometry/Pose2.h>
#include "general/debug_utils.hpp"
#define BOOST_AUTO_TEST_MAIN constants_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/general/param_loader.hpp"
#include "prx/utilities/general/gtsam_bridge.hpp"

BOOST_AUTO_TEST_CASE(test_rot2)
{
  prx::param_loader param;
  param["rot2/theta"] = 0.5;
  gtsam::Rot2 r{ param["rot2"].as<gtsam::Rot2>() };

  BOOST_REQUIRE(r.theta() == 0.5);
}

BOOST_AUTO_TEST_CASE(test_pose2)
{
  prx::param_loader param;
  param["pose2/x"] = 1.0;
  param["pose2/y"] = 2.0;
  param["pose2/theta"] = 3.0;
  gtsam::Pose2 p{ param["pose2"].as<gtsam::Pose2>() };

  BOOST_REQUIRE(p.x() == 1.0);
  BOOST_REQUIRE(p.y() == 2.0);
  BOOST_REQUIRE(p.theta() == 3.0);
}

BOOST_AUTO_TEST_CASE(test_rot3_quat)
{
  prx::param_loader param;
  param["rot3/w"] = 0.707;
  param["rot3/x"] = 0.0;
  param["rot3/y"] = 0.707;
  param["rot3/z"] = 0.0;
  gtsam::Rot3 p{ param["rot3"].as<gtsam::Rot3>() };

  Eigen::Quaterniond q{ p.toQuaternion() };
  BOOST_REQUIRE(std::fabs(q.w() - 0.707) < 0.01);
  BOOST_REQUIRE(std::fabs(q.x()) < 0.001);
  BOOST_REQUIRE(std::fabs(q.y() - 0.707) < 0.01);
  BOOST_REQUIRE(std::fabs(q.z()) < 0.001);
}

BOOST_AUTO_TEST_CASE(test_rot3_mat)
{
  prx::param_loader param;
  const std::vector<double> mat_vec{ { 0.000302, 0, 0.999698, 0, 1, 0, -0.999698, 0, 0.000302 } };
  param["rot3"] = mat_vec;
  gtsam::Rot3 p{ param["rot3"].as<gtsam::Rot3>() };

  Eigen::Quaterniond q{ p.toQuaternion() };
  BOOST_REQUIRE(std::fabs(q.w() - 0.707) < 0.01);
  BOOST_REQUIRE(std::fabs(q.x()) < 0.001);
  BOOST_REQUIRE(std::fabs(q.y() - 0.707) < 0.01);
  BOOST_REQUIRE(std::fabs(q.z()) < 0.001);
}

BOOST_AUTO_TEST_CASE(test_pose3_mat)
{
  prx::param_loader param;
  const std::vector<double> mat_vec{ { 0.000302, 0, 0.999698, 0, 1, 0, -0.999698, 0, 0.000302 } };
  const std::vector<double> vec{ { 1.0, 2.0, 3.0 } };
  param["pose3/rotation"] = mat_vec;
  param["pose3/translation"] = vec;
  gtsam::Pose3 p{ param["pose3"].as<gtsam::Pose3>() };

  Eigen::Vector3d t{ p.translation() };
  Eigen::Quaterniond q{ p.rotation().toQuaternion() };
  BOOST_REQUIRE(std::fabs(q.w() - 0.707) < 0.01);
  BOOST_REQUIRE(std::fabs(q.x()) < 0.001);
  BOOST_REQUIRE(std::fabs(q.y() - 0.707) < 0.01);
  BOOST_REQUIRE(std::fabs(q.z()) < 0.001);
  BOOST_REQUIRE(std::fabs(t[0] - 1.0) < 0.001);
  BOOST_REQUIRE(std::fabs(t[1] - 2.0) < 0.001);
  BOOST_REQUIRE(std::fabs(t[2] - 3.0) < 0.001);
}

BOOST_AUTO_TEST_CASE(test_product_lie)
{
  using ProductLieGroup = gtsam::ProductLieGroup<gtsam::Pose2, Eigen::Vector3d>;
  prx::param_loader param;

  param["pose2_vel/first/x"] = 1.0;
  param["pose2_vel/first/y"] = 2.0;
  param["pose2_vel/first/theta"] = 3.0;
  param["pose2_vel/second"] = std::vector<double>({ 0.1, 0.2, 0.3 });
  ProductLieGroup p{ param["pose2_vel"].as<ProductLieGroup>() };

  BOOST_REQUIRE(p.first.x() == 1.0);
  BOOST_REQUIRE(p.first.y() == 2.0);
  BOOST_REQUIRE(p.first.theta() == 3.0);

  BOOST_REQUIRE(p.second[0] == 0.1);
  BOOST_REQUIRE(p.second[1] == 0.2);
  BOOST_REQUIRE(p.second[2] == 0.3);
}