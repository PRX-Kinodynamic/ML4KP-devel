#ifndef TORCH_NOT_BUILT
#pragma once

#include "prx/simulation/controller.hpp"
#include "prx/simulation/plants/types/linear_time_variant.hpp"
#include "prx/simulation/plants/types/linear_time_invariant.hpp"
#include "prx/utilities/math/continuous_algebraic_riccati_equation.hpp"

#include <torch/torch.h>
#include <torch/script.h>

namespace prx
{
class torch_controller_t : public controller_t
{
public:
  // template<class S>
  torch_controller_t(const system_ptr_t& _sys_ptr, std::string _name, torch::DeviceType type, long long num_predictions,
                     long long input_size, std::size_t xi_offset, std::size_t xgi_offset,
                     const std::string network_path)
    : controller_t(_sys_ptr, _name)
    , _num_predictions(num_predictions)
    , _torch_device(type)
    , _xi_offset(xi_offset)
    , _xgi_offset(xgi_offset)

  {
    // const long long udim{ static_cast<long long>(plant->get_control_space()->get_dimension()) };
    u = plant->get_control_space()->make_point();
    goal = plant->get_state_space()->make_point();
    _inputs = torch::zeros({ _num_predictions, input_size }, _torch_device);
    torch::set_num_threads(1);
    try
    {
      controller = torch::jit::load(network_path, _torch_device);
      // torch::load(controller, network_path, _torch_device);
      // torch::load(controller, network_path);
    }
    catch (const c10::Error& e)
    {
      std::cerr << e.msg() << std::endl;
      std::cerr << "[torch_controller_t] Error loading the model: " << network_path << std::endl;
      exit(-1);
    }
  }

  virtual ~torch_controller_t()
  {
  }

  using controller_t::compute_controls;
  virtual void compute_controls() override
  {
    const std::size_t x_dim{ plant->get_state_space()->get_dimension() };
    for (int i = 0; i < _num_predictions; ++i)
    {
      for (int xi = 0; xi < x_dim; ++xi)
      {
        _inputs[i][_xi_offset + xi] = plant->get_state_space()->at(xi);
      }
      for (int xgi = 0; xgi < x_dim; ++xgi)
      {
        _inputs[i][_xgi_offset + xgi] = goal->at(xgi);
      }
    }

    _controller_output = controller.forward({ _inputs }).toTensor();
    get_output(*u, 0);
    plant->get_control_space()->copy_from(u);
    plant->get_control_space()->enforce_bounds();
  }

  template <typename T>
  void get_output(T& to, const std::size_t& idx)
  {
    const space_t* u_space = plant->get_control_space();
    const std::size_t u_dim{ u_space->get_dimension() };
    for (int ui = 0; ui < u_dim; ++ui)
    {
      to[ui] = (u_space->get_lower_bound(ui)) +
               (_controller_output[idx][ui].item<double>() + 1) * (u_space->get_upper_bound(ui));
    }
  }

protected:
  space_point_t u;
  torch::Device _torch_device;
  long long _num_predictions;
  at::Tensor _inputs;
  at::Tensor _controller_output;
  std::size_t _xi_offset;
  std::size_t _xgi_offset;
  torch::jit::script::Module controller;
  // std::vector<torch::jit::IValue> _inputs;
};
}  // namespace prx
#endif