#ifndef TORCH_NOT_BUILT

#include <torch/torch.h>
#include <torch/script.h>  // One-stop header.

#include <iostream>
#include <memory>
#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/param_loader.hpp"

int main(int argc, char* argv[])
{
  prx::param_loader params{};
  params.add_opts(argc, argv);

  // torch::jit::script::Module module;

  torch::Tensor x0{ torch::ones({ 1 }) };
  torch::Tensor x1{ 2.0 * torch::ones({ 1 }) };
  x0.requires_grad_();
  x1.requires_grad_();

  torch::Tensor y0{ x0 * x0 * x1 };
  // return y0;
  torch::Tensor y1{ 5.0 * x0 + at::sin(x1) };
  torch::Tensor y01{ torch::hstack({ y0, y1 }) };
  PRX_DBG_VARS(y01);

  torch::Tensor dy0dx0{ 2.0 * x0 * x1 };
  torch::Tensor dy0dx1{ x0 * x0 };
  torch::Tensor dy1dx0{ 5.0 * torch::ones({ 1 }) };
  torch::Tensor dy1dx1{ at::cos(x1) };
  torch::Tensor dydx{ torch::vstack({ torch::hstack({ dy0dx0, dy0dx1 }), torch::hstack({ dy1dx0, dy1dx1 }) }) };
  PRX_DBG_VARS(dy0dx0);
  PRX_DBG_VARS(dy0dx1);
  PRX_DBG_VARS(dy1dx0);
  PRX_DBG_VARS(dy1dx1);
  PRX_DBG_VARS(dydx);
  try
  {
    // module = torch::jit::load(params["model"].as<>());

    // torch::Tensor nn_output, loss;
    for (int i = 0; i < 1; ++i)
    {
      // std::vector<torch::jit::IValue> inputs;
      // torch::Tensor x{ torch::zeros({ 1 }, torch::requires_grad()) };
      // torch::Tensor u{ torch::zeros({ 1 }, torch::requires_grad()) };
      // inputs.push_back(x);
      // inputs.push_back(u);

      // std::vector<double> e{ 1, 0, 0 };
      // auto x = torch::from_blob(e.data(), 3, torch::requires_grad());
      // auto f = dot(x, x);
      // std::cout << "x = \n" << x << std::endl;
      // std::cout << "f = dot(x,x) = " << f << std::endl;

      torch::Tensor one{ torch::ones({ 1 }) };
      torch::Tensor zero{ torch::zeros({ 1 }) };
      torch::Tensor grad10{ torch::hstack({ one, zero }) };
      torch::Tensor grad01{ torch::hstack({ zero, one }) };
      torch::Tensor grad11{ torch::hstack({ one, one }) };
      // torch::Tensor eye{ torch::eye(2) };
      // PRX_DBG_VARS(grad);
      auto p_dy0dx0 = torch::autograd::grad({ y01 }, { x0 }, { grad10 }, true);
      auto p_dy1dx0 = torch::autograd::grad({ y01 }, { x0 }, { grad01 }, true);
      auto p_dy0dx1 = torch::autograd::grad({ y01 }, { x1 }, { grad10 }, true);
      auto p_dy1dx1 = torch::autograd::grad({ y01 }, { x1 }, { grad01 }, true);

      torch::Tensor p_dydx{ torch::vstack(
          { torch::hstack({ p_dy0dx0[0], p_dy0dx1[0] }), torch::hstack({ p_dy1dx0[0], p_dy1dx1[0] }) }) };
      PRX_DBG_VARS(p_dy0dx0);
      PRX_DBG_VARS(p_dy1dx0);
      PRX_DBG_VARS(p_dy0dx1);
      PRX_DBG_VARS(p_dy1dx1);
      PRX_DBG_VARS(p_dydx);

      auto p_dyidxi = torch::autograd::grad({ y01, y01 }, { x0, x1 }, { grad10, grad01 }, true);
      PRX_DBG_VARS(p_dyidxi);

      // torch::autograd::backward({ y01 }, { grad11 }, true, false, { x0 });
      // auto partial_f01_x = torch::autograd::grad({ y01 }, { x0 }, { grad01 }, true);
      // PRX_DBG_VARS(backward_f10_x);
      // partial_f10_x[0].backward();
      // PRX_DBG_VARS(x0.grad());
      // PRX_DBG_VARS(partial_f01_x);
      // std::cout << "partial_f_x = \n" << partial_f_x << std::endl;

      // nn_output = module.forward(inputs).toTensor();

      // nn_output.backward();
      // PRX_DBG_VARS(nn_output);
      // PRX_DBG_VARS(x.grad());
      // PRX_DBG_VARS(u.grad());
    }
  }
  catch (const c10::Error& e)
  {
    std::cerr << "Error loading the model\n";
    std::cout << e.what() << "\n";

    return -1;
  }

  return 0;
}
#else
int main()
{
}
#endif
