#ifndef TORCH_NOT_BUILT

#include <torch/script.h>  // One-stop header.

#include <iostream>
#include <memory>
#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/param_loader.hpp"

int main(int argc, char* argv[])
{
  prx::param_loader params{};
  params.add_opts(argc, argv);

  torch::jit::script::Module module;
  try
  {
    // Deserialize the ScriptModule from a file using torch::jit::load().
    module = torch::jit::load(params["model"].as<>());

    // Create a vector of inputs.
    // std::vector<int> vin{ params["inputs"].as<std::vector<int>>() };

    // torch::Tensor input = torch::zeros({ 2 });
    torch::Tensor nn_output, loss;
    for (int i = 0; i < 10; ++i)
    {
      std::vector<torch::jit::IValue> inputs;
      torch::Tensor x{ torch::zeros({ 1 }, torch::requires_grad()) };
      torch::Tensor u{ torch::zeros({ 1 }, torch::requires_grad()) };
      inputs.push_back(x);
      inputs.push_back(u);
      /* code */
      // Execute the model and turn its output into a tensor.
      // const at::Tensor nn_output{ module.forward(inputs).toTensor() };
      nn_output = module.forward(inputs).toTensor();
      // nn_output = module.forward(inputs, inputs).toTensor();
      // module.backward(inputs);
      // std::cout << output.slice(/*dim=*/1, /*start=*/0, /*end=*/5) << '\n';
      // std::cout << output << '\n';
      nn_output.backward();
      PRX_DBG_VARS(nn_output);
      // PRX_DBG_VARS(nn_output.grad_fn()->name());
      PRX_DBG_VARS(x.grad());
      PRX_DBG_VARS(u.grad());
      // PRX_DBG_VARS(inputs[0].grad());
      // nn_output.backward();
    }
  }
  catch (const c10::Error& e)
  {
    std::cerr << "Error loading the model\n";
    std::cout << e.what() << "\n";

    return -1;
  }

  // std::cout << "ok\n";
  return 0;
}
#else
int main()
{
}
#endif
