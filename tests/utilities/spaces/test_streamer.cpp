#define BOOST_AUTO_TEST_MAIN spaces_test

#include <chrono>
#include <string>

#include <boost/test/unit_test.hpp>
#include "prx/utilities/spaces/streamer.hpp"

template <typename Type>
void mock_stream(std::ostream& os, const Type& t)
{
  prx::streamer_t<Type>::to_stream(os, t);
}

BOOST_AUTO_TEST_CASE(test_streamer_general)
{
  std::stringstream strstr;

  mock_stream(strstr, "test");

  BOOST_CHECK(strstr.str() == "test ");
}

BOOST_AUTO_TEST_CASE(test_streamer_eigen_vector)
{
  std::stringstream strstr;

  const Eigen::Vector2d vec{ Eigen::Vector2d::Ones() };
  mock_stream(strstr, vec);

  BOOST_CHECK(strstr.str() == "1 1 ");
}

BOOST_AUTO_TEST_CASE(test_streamer_gtsam_pose2)
{
  std::stringstream strstr;

  const gtsam::Pose2 p(0, 1, 2);
  mock_stream(strstr, p);

  BOOST_CHECK(strstr.str() == "0 1 2 ");
}

BOOST_AUTO_TEST_CASE(test_streamer_streamable)
{
  std::stringstream strstr;

  const std::vector<int> p = { 0, 1, 2 };
  mock_stream(strstr, p);

  BOOST_CHECK(strstr.str() == "0 \n1 \n2 \n");
}

BOOST_AUTO_TEST_CASE(test_streamer_string)
{
  std::stringstream strstr;

  // const std::vector<int> p = { 0, 1, 2 };
  const std::string test{ "test" };
  mock_stream(strstr, test);

  BOOST_CHECK(strstr.str() == "test ");
}

BOOST_AUTO_TEST_CASE(test_streamer_path)
{
  std::stringstream strstr;

  std::filesystem::path p("test");

  mock_stream(strstr, p);
  // PRX_DBG_VARS(strstr.str())
  BOOST_CHECK(strstr.str() == "\"test\" ");
}

BOOST_AUTO_TEST_CASE(test_eigen_matrix)
{
  std::stringstream strstr;

  Eigen::Matrix2d m(Eigen::Matrix2d::Identity());

  mock_stream(strstr, m);
  // PRX_DBG_VARS(strstr.str())
  BOOST_CHECK(strstr.str() == "1 0\n0 1 ");
}

BOOST_AUTO_TEST_CASE(test_product_lie_group)
{
  std::stringstream strstr;

  gtsam::Pose2 pose(0, 1, 2);
  Eigen::Vector3d vec(3, 4, 5);
  gtsam::ProductLieGroup<gtsam::Pose2, Eigen::Vector3d> x(pose, vec);

  mock_stream(strstr, x);
  PRX_DBG_VARS(strstr.str())
  BOOST_CHECK(strstr.str() == "0 1 2 3 4 5 ");
}

BOOST_AUTO_TEST_CASE(test_product_lie_groupV43)
{
  std::stringstream strstr;

  gtsam::Pose2 pose(0, 1, 2);
  Eigen::Vector3d vec(3, 4, 5);
  gtsam::ProductLieGroupV43<gtsam::Pose2, Eigen::Vector3d> x(pose, vec);

  mock_stream(strstr, x);
  PRX_DBG_VARS(strstr.str())
  BOOST_CHECK(strstr.str() == "0 1 2 3 4 5 ");
}

BOOST_AUTO_TEST_CASE(test_product_vector_lie_groupV43)
{
  std::stringstream strstr;

  gtsam::Pose2 pose(0, 1, 2);
  Eigen::Vector3d vec(3, 4, 5);
  std::vector<gtsam::ProductLieGroupV43<gtsam::Pose2, Eigen::Vector3d>> vector;  //(pose, vec);

  vector.emplace_back(pose, vec);
  vector.emplace_back(pose, vec);
  mock_stream(strstr, vector);
  PRX_DBG_VARS(strstr.str())
  BOOST_CHECK(strstr.str() == "0 1 2 3 4 5 \n0 1 2 3 4 5 \n");
}
