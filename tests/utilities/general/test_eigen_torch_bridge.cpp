#define BOOST_AUTO_TEST_MAIN quaternion_test
#include <boost/test/unit_test.hpp>
#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/torch_eigen_bridge.hpp"

BOOST_AUTO_TEST_CASE(eigen_vec_to_torch_tensor)
{
  Eigen::Vector2d vec{ Eigen::Vector2d(1, 2) };
  torch::Tensor tensor{ torch::zeros({ 2 }) };
  prx::copy(tensor, vec);

  BOOST_REQUIRE(tensor[0].item<float>() == 1.0);
  BOOST_REQUIRE(tensor[1].item<float>() == 2.0);
}
BOOST_AUTO_TEST_CASE(eigen_vec_to_torch_sliced_tensor)
{
  Eigen::Vector4d vec4{ Eigen::Vector4d(1, 2, 3, 4) };
  torch::Tensor tensor{ torch::zeros({ 4 }) };
  prx::copy(tensor, vec4);
  // PRX_DBG_VARS(tensor);
  torch::Tensor tensor01{ tensor.index({ torch::indexing::Slice(0, 2) }) };
  torch::Tensor tensor23{ tensor.index({ torch::indexing::Slice(2, 4) }) };

  prx::copy(tensor01, vec4.tail(2));
  prx::copy(tensor23, vec4.head(2));

  // PRX_DBG_VARS(tensor);
  // PRX_DBG_VARS(tensor01);

  // Reverse order of vec4 in chunks of 2: [3,4,  1,2]
  BOOST_REQUIRE(tensor[0].item<float>() == 3.0);
  BOOST_REQUIRE(tensor[1].item<float>() == 4.0);
  BOOST_REQUIRE(tensor[2].item<float>() == 1.0);
  BOOST_REQUIRE(tensor[3].item<float>() == 2.0);
}

BOOST_AUTO_TEST_CASE(torch_tensor_to_eigen_vec)
{
  torch::Tensor tensor{ torch::arange(1.0, 3.0, 1) };
  PRX_DBG_VARS(tensor);
  Eigen::Vector2d vec{ Eigen::Vector2d::Zero() };
  prx::copy(vec, tensor);

  BOOST_REQUIRE(vec[0] == 1.0);
  BOOST_REQUIRE(vec[1] == 2.0);
}