#define BOOST_AUTO_TEST_MAIN torch_interface_test
#include <boost/test/unit_test.hpp>
#ifndef TORCH_NOT_BUILT
#include <string>
#include "prx/utilities/general/torch_interface.hpp"
#include "prx/utilities/general/debug_utils.hpp"
#include "prx/utilities/defs.hpp"
#include <boost/optional.hpp>
#include <torch/torch.h>

namespace mock
{
struct torch_nn_2out_1in_t
{
  torch_nn_2out_1in_t()
  {
  }
  torch::jit::IValue forward(std::vector<torch::jit::IValue>& xin)
  {
    torch::Tensor x0{ xin[0].toTensor() };
    x0.requires_grad_();
    torch::Tensor y0{ 5.0 * x0 + at::sin(x0) };
    // return y0;
    torch::Tensor y1{ 5.0 * x0 * x0 };
    torch::Tensor y01{ torch::hstack({ y0, y1 }) };
    PRX_DBG_VARS(y01);
    return y01;
  }

  Eigen::Matrix<double, 2, 1> deriv(Eigen::Vector<double, 1>& x)
  {
    Eigen::Matrix<double, 2, 1> dv;
    dv(0, 0) = std::cos(x[0]) + 5.0;
    dv(1, 0) = 10.0 * x[0];

    return dv;
  }
};

struct torch_nn_3out_2in_t
{
  torch_nn_3out_2in_t()
  {
  }
  torch::jit::IValue forward(std::vector<torch::jit::IValue>& xin)
  {
    // PRX_DEBUG_PRINT
    // PRX_DBG_VARS(xin)
    // PRX_DBG_VARS(xin[0])
    // PRX_DBG_VARS(xin.size())
    torch::Tensor x{ xin[0].toTensor() };
    torch::Tensor x0{ x.index({ 0 }) };
    // PRX_DBG_VARS(x0);
    torch::Tensor x1{ x.index({ 1 }) };
    // PRX_DBG_VARS(x1);
    x0.requires_grad_();
    x1.requires_grad_();
    torch::Tensor y0{ 5.0 * x0 + at::sin(x1) };
    torch::Tensor y1{ 5.0 * x0 * x1 };
    torch::Tensor y2{ 10.0 * x1 * x1 / (1.0 + at::cos(x0)) };
    torch::Tensor y{ torch::hstack({ y0, y1, y2 }) };
    // PRX_DBG_VARS(y);
    return y;
  }

  Eigen::Matrix<double, 3, 2> deriv(Eigen::Vector2d x)
  {
    // From matlab:
    // [                                5,               cos(x1)]
    // [                             5*x1,                  5*x0]
    // [(10*x1^2*sin(x0))/(cos(x0) + 1)^2, (20*x1)/(cos(x0) + 1)]
    Eigen::Matrix<double, 3, 2> dv;
    dv(0, 0) = 5.0;
    dv(1, 0) = 5.0 * x[1];
    dv(2, 0) = 10.0 * x[1] * x[1] * std::sin(x[0]) / std::pow(std::cos(x[0]) + 1, 2);

    dv(0, 1) = std::cos(x[1]);
    dv(1, 1) = 5.0 * x[0];
    dv(2, 1) = (20.0 * x[1]) / (std::cos(x[0]) + 1.0);

    return dv;
  }
};

struct torch_nn_3out_1a1in_t
{
  torch_nn_3out_1a1in_t()
  {
  }
  torch::jit::IValue forward(std::vector<torch::jit::IValue>& xin)
  {
    // PRX_DEBUG_PRINT
    // PRX_DBG_VARS(xin)
    // PRX_DBG_VARS(xin[0])
    // PRX_DBG_VARS(xin.size())
    torch::Tensor x0{ xin[0].toTensor() };
    torch::Tensor x1{ xin[1].toTensor() };
    // torch::Tensor x0{ x.index({ 0 }) };
    // PRX_DBG_VARS(x0);
    // torch::Tensor x1{ x.index({ 1 }) };
    // PRX_DBG_VARS(x1);
    x0.requires_grad_();
    x1.requires_grad_();
    torch::Tensor y0{ 5.0 * x0 + at::sin(x1) };
    torch::Tensor y1{ 5.0 * x0 * x1 };
    torch::Tensor y2{ 10.0 * x1 * x1 / (1.0 + at::cos(x0)) };
    torch::Tensor y{ torch::hstack({ y0, y1, y2 }) };
    // PRX_DBG_VARS(y);
    return y;
  }

  Eigen::Matrix<double, 3, 1> deriv_0(Eigen::Vector<double, 1> x0, Eigen::Vector<double, 1> x1)
  {
    // From matlab:
    // [                                5,               cos(x1)]
    // [                             5*x1,                  5*x0]
    // [(10*x1^2*sin(x0))/(cos(x0) + 1)^2, (20*x1)/(cos(x0) + 1)]
    Eigen::Matrix<double, 3, 1> dv;
    dv(0, 0) = 5.0;
    dv(1, 0) = 5.0 * x1[0];
    dv(2, 0) = 10.0 * x1[0] * x1[0] * std::sin(x0[0]) / std::pow(std::cos(x0[0]) + 1, 2);
    return dv;
  }
  Eigen::Matrix<double, 3, 1> deriv_1(Eigen::Vector<double, 1> x0, Eigen::Vector<double, 1> x1)
  {
    Eigen::Matrix<double, 3, 1> dv;
    dv(0, 0) = std::cos(x1[0]);
    dv(1, 0) = 5.0 * x0[0];
    dv(2, 0) = (20.0 * x1[0]) / (std::cos(x0[0]) + 1.0);

    return dv;
  }
};
}  // namespace mock

BOOST_AUTO_TEST_CASE(eval_no_derivs_test)
{
  using Input = Eigen::Vector<double, 1>;
  using Output = Eigen::Vector<double, 2>;
  using TorchInterface = prx::utilities::torch_nn_interface_t<mock::torch_nn_2out_1in_t, Output, Input>;
  // PRX_DEBUG_PRINT
  std::shared_ptr<mock::torch_nn_2out_1in_t> model{ std::make_shared<mock::torch_nn_2out_1in_t>() };
  // std::vector<torch::jit::IValue> input;
  Input x0{ Input::Ones() };
  // torch::Tensor x{ torch::zeros({ 1 }, torch::requires_grad()) };
  // input.push_back(x);
  // prx::utilities::torch_nn_interface_t<mock::torch_nn_2out_1in_t, Output, Input> interface(model);
  TorchInterface interface(model);
  Output y0{ interface.eval(x0) };

  Output y0_expected{ Output(5.8414709848, 5.0) };
  // torch::Tensor y0_expected({ 4.207354924, 5.0 });
  BOOST_CHECK(y0.isApprox(y0_expected, 1e-3));
}

BOOST_AUTO_TEST_CASE(eval_with_derivs_test)
{
  using Input = Eigen::Vector<double, 1>;
  using Output = Eigen::Vector<double, 2>;
  using TorchInterface = prx::utilities::torch_nn_interface_t<mock::torch_nn_2out_1in_t, Output, Input>;

  std::shared_ptr<mock::torch_nn_2out_1in_t> model{ std::make_shared<mock::torch_nn_2out_1in_t>() };
  Input x0{ Input::Ones() };

  Eigen::MatrixXd deriv;
  Eigen::MatrixXd expected_deriv{ model->deriv(x0) };
  TorchInterface interface(model);
  Output y0{ interface.eval(x0, deriv) };

  // out(0, 0) = 5.0 + std::cos(x);
  // out(1, 0) = 2.0 * 5.0 * x;
  BOOST_CHECK(deriv.isApprox(expected_deriv, 1e-3));
}

BOOST_AUTO_TEST_CASE(eval32_with_deriv_test)
{
  using Input = Eigen::Vector<double, 2>;
  using Output = Eigen::Vector<double, 3>;
  using TorchInterface = prx::utilities::torch_nn_interface_t<mock::torch_nn_3out_2in_t, Output, Input>;

  std::shared_ptr<mock::torch_nn_3out_2in_t> model{ std::make_shared<mock::torch_nn_3out_2in_t>() };
  Input x0{ Input(1, 2) };

  // PRX_DBG_VARS(x0)
  Eigen::MatrixXd deriv;
  Eigen::MatrixXd expected_deriv{ model->deriv(x0) };
  // PRX_DBG_VARS(expected_deriv)
  TorchInterface interface(model);
  Output y{ interface.eval(x0, deriv) };

  // 5.0 * x0 + sin(x1)
  // 5.0 * x0 * x1
  // 10.0 * x1 * x1 / (1.0 + cos(x0))
  Output y_expected{ Output(5.9092974268, 10.0, 25.9689282082) };

  // PRX_DBG_VARS(expected_deriv);
  // out(0, 0) = 5.0 + std::cos(x);
  // out(1, 0) = 2.0 * 5.0 * x;
  BOOST_CHECK(y_expected.isApprox(y, 1e-3));
  BOOST_CHECK(expected_deriv.isApprox(deriv, 1e-3));
}

BOOST_AUTO_TEST_CASE(eval311_with_deriv_test)
{
  using Input = Eigen::Vector<double, 1>;
  using Output = Eigen::Vector<double, 3>;
  using TorchInterface = prx::utilities::torch_nn_interface_t<mock::torch_nn_3out_1a1in_t, Output, Input, Input>;

  std::shared_ptr<mock::torch_nn_3out_1a1in_t> model{ std::make_shared<mock::torch_nn_3out_1a1in_t>() };
  Input x0{ Input(1) };
  Input x1{ Input(2) };

  // PRX_DBG_VARS(x0)
  Eigen::MatrixXd deriv0, deriv1;
  Eigen::MatrixXd expected_deriv0{ model->deriv_0(x0, x1) };
  Eigen::MatrixXd expected_deriv1{ model->deriv_1(x0, x1) };
  // PRX_DBG_VARS(expected_deriv)
  TorchInterface interface(model);
  Output y{ interface.eval(x0, x1, deriv0, deriv1) };

  // 5.0 * x0 + sin(x1)
  // 5.0 * x0 * x1
  // 10.0 * x1 * x1 / (1.0 + cos(x0))
  Output y_expected{ Output(5.9092974268, 10.0, 25.9689282082) };

  // PRX_DBG_VARS(deriv1);
  // PRX_DBG_VARS(expected_deriv1);
  // out(0, 0) = 5.0 + std::cos(x);
  // out(1, 0) = 2.0 * 5.0 * x;
  BOOST_CHECK(y_expected.isApprox(y, 1e-3));
  BOOST_CHECK(expected_deriv0.isApprox(deriv0, 1e-3));
  BOOST_CHECK(expected_deriv1.isApprox(deriv1, 1e-3));
}
#else
BOOST_AUTO_TEST_CASE(torch_interface_not_built)
{
  std::cout << "TORCH_NOT_BUILT" << std::endl;
}
#endif