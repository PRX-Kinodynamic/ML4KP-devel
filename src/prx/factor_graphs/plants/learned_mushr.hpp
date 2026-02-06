#ifndef TORCH_NOT_BUILT
#pragma once
#include <torch/script.h>  // One-stop header.
#include "prx/simulation/plant.hpp"
#include "prx/factor_graphs/factors/euler_integration_factor.hpp"
#include "prx/factor_graphs/lie_groups/lie_integrator.hpp"
#include "prx/utilities/general/torch_interface.hpp"
#include "prx/factor_graphs/lie_groups/se2.hpp"

namespace prx
{
namespace fg
{

class learned_mushr_t : public plant_t
{
public:
  using State = prx::fg::SE2_t;
  using Velocity = Eigen::Vector<double, 3>;
  using Acceleration = Eigen::Vector<double, 3>;
  using Control = Eigen::Vector<double, 2>;
  using LieIntegrator = prx::fg::lie_integrator_t<State, Velocity, double>;
  using EulerIntegrator = prx::fg::euler_integration_factor_t<Velocity, Acceleration, double>;

  using TorchModule = torch::jit::script::Module;
  using TorchModulePtr = std::shared_ptr<TorchModule>;
  using TorchInterface = prx::utilities::torch_nn_interface_t<TorchModule, Acceleration, Velocity, Control>;

  learned_mushr_t(const std::string& path) : plant_t(path), _model_initialized(false)
  {
    state_memory = { &_x[0], &_x[1], &_x[2], &_xd[0], &_xd[1], &_xd[2] };
    state_space = new space_t("EEREEE", state_memory, "learned_mushr_state");
    // state_space->set_bounds({ -prx::constants::pi, -2.0 * prx::constants::pi },
    //                         { +prx::constants::pi, +2.0 * prx::constants::pi });

    control_memory = { &_u[0], &_u[1] };
    input_control_space = new space_t("EE", control_memory, "Torque");
    // input_control_space->set_bounds({ -0.6371781908344007 }, { 0.6371781908344007 });

    derivative_memory = { &_xd[0], &_xd[1], &_xd[2], &_xdd[0], &_xdd[1], &_xdd[2] };
    derivative_space = new space_t("EEEEEE", derivative_memory, "mushr_deriv");

    geometries["body"] = std::make_shared<prx::geometry_t>(prx::geometry_type_t::BOX);
    geometries["body"]->initialize_geometry({ 0.42, 0.25, 0.25 });
    geometries["body"]->generate_collision_geometry();
    geometries["body"]->set_visualization_color("0x00ff00");
    configurations["body"] = std::make_shared<prx::transform_t>();
    configurations["body"]->setIdentity();

    parameter_memory = {};
    parameter_space = new space_t("", parameter_memory, "params");
  }

  virtual ~learned_mushr_t()
  {
  }

  virtual void propagate(const double simulation_step) override final
  {
    prx_assert(_model_initialized, "Torch model has not been initialized");
    // _xdd = _nn(_xd, _u);
    _xdd = _nn(_xd, _u);
    // _xdd[0] = -xdd[1];
    // _xdd[1] = xdd[0];

    _xd = EulerIntegrator::integrate(_xd, _xdd, simulation_step);
    _x = LieIntegrator::integrate(_x, _xd, simulation_step);
  }

  virtual void update_configuration() override
  {
    auto body = configurations["body"];
    body->linear() = Eigen::Matrix3d{ Eigen::AngleAxisd(_x[2], Eigen::Vector3d::UnitZ()) };
    body->translation()[0] = _x[0];
    body->translation()[1] = _x[1];
    body->translation()[2] = 0.0;
  }

  virtual void compute_derivative() override
  {
  }

  virtual void init(const prx::param_loader& params) override
  {
    prx::plant_t::init(params);
    if (params.exists("torch_file"))
    {
      const std::string torch_filename{ params["torch_file"].as<>() };
      PRX_DBG_VARS(torch_filename);
      _nn.load_model(torch_filename);
      _model_initialized = true;
    }
  }

  virtual prx::param_loader init() override
  {
    prx::param_loader params{ prx::plant_t::init() };
    params["torch_file"].set("");

    return params;
  }

protected:
  State _x;
  Velocity _xd;
  Acceleration _xdd;
  Control _u;

  TorchInterface _nn;
  bool _model_initialized;
};
}  // namespace fg
}  // namespace prx

PRX_REGISTER_SYSTEM(fg::learned_mushr_t, learned_mushr)
#endif
