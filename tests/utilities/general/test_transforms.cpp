#define BOOST_AUTO_TEST_MAIN transforms_test
#include <string>
#include <boost/test/unit_test.hpp>

#include "prx/utilities/general/transforms.hpp"

BOOST_AUTO_TEST_CASE(init_from_container_initializes_matrix_correctly)
{
  Eigen::Matrix3d mat_to_init;
  std::vector<double> vector_of_values = { 0, 1, 2, 3, 4, 5, 6, 7, 8 };
  prx::init_from_container(mat_to_init, vector_of_values);

  Eigen::Matrix3d expected_mat;
  expected_mat << 0, 1, 2, 3, 4, 5, 6, 7, 8;

  BOOST_CHECK_MESSAGE(mat_to_init == expected_mat, "Expected: " << expected_mat << ", Got " << mat_to_init);
}