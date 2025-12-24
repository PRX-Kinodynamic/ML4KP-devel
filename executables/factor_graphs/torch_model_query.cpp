#include <torch/script.h>  // One-stop header.

#include <iostream>
#include <memory>
#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/utilities/general/torch_interface.hpp"

using TorchModule = torch::jit::script::Module;
using TorchInterface =
    prx::utilities::torch_nn_interface_t<TorchModule, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector2d>;

int main(int argc, char* argv[])
{
  prx::param_loader params{};
  params.add_opts(argc, argv);

  const std::string torch_filename{ params["model"].as<>() };

  TorchInterface nn;

  Eigen::Vector3d x, out;
  Eigen::Vector2d u;

  x = params["x"].as<Eigen::Vector3d>();
  u = params["u"].as<Eigen::Vector2d>();
  try
  {
    PRX_DBG_VARS(torch_filename);
    nn.load_model(torch_filename);

    PRX_DBG_VARS(x.transpose(), u.transpose());
    out = nn(x, u);
    PRX_DBG_VARS(out.transpose());
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