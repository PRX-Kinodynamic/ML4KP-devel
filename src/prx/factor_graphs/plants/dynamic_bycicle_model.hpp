#ifndef TORCH_NOT_BUILT
#pragma once
#include <torch/script.h>  // One-stop header.

#include "prx/simulation/plant.hpp"
#include "prx/factor_graphs/factors/euler_integration_factor.hpp"
#include "prx/factor_graphs/lie_groups/lie_integrator.hpp"
#include "prx/utilities/general/torch_interface.hpp"
#include "prx/factor_graphs/lie_groups/se2.hpp"
#include "prx/factor_graphs/utilities/gtsam_extra_utilities.hpp"
#include "prx/factor_graphs/factors/dynamic_bicycle_factors.hpp"
namespace prx
{
namespace fg
{

// Model from:
// Kabzan, Juraj, Lukas Hewing, Alexander Liniger, and Melanie N. Zeilinger.
// "Learning-based model predictive control for autonomous racing."
// IEEE Robotics and Automation Letters 4, no. 4 (2019): 3363-3370.

class dynamic_vehicle_t : public plant_t
{
public:
  using BikeDynamics = bike_dynamics_t;
  using Params = bike_dynamics_t::Params;
  using StaticParams = bike_dynamics_t::StaticParams;
  using State = prx::fg::SE2_t;
  using Velocity = Eigen::Vector<double, 3>;
  using Acceleration = Eigen::Vector<double, 3>;
  using Control = Eigen::Vector<double, 2>;
  using LieIntegrator = prx::fg::lie_integrator_t<State, Velocity, double>;
  using EulerIntegrator = prx::fg::euler_integration_factor_t<Velocity, Acceleration, double>;

  using TorchModule = torch::jit::script::Module;
  using TorchModulePtr = std::shared_ptr<TorchModule>;
  using TorchInterface = prx::utilities::torch_nn_interface_t<TorchModule, Acceleration, Velocity, Control>;

  dynamic_vehicle_t(const std::string& path)
    : plant_t(path)
    , _model_initialized(false)
    , _params(Params::Ones())
    , _static_params(BikeDynamics::init_static_params(3.5, 0.155, 0.155))
  {
    state_memory = { &_x[0], &_x[1], &_x[2], &_xd[0], &_xd[1], &_xd[2] };
    state_space = new space_t("EEREEE", state_memory, "dynamic_vehicle");
    // state_space->set_bounds({ -prx::constants::pi, -2.0 * prx::constants::pi },
    //                         { +prx::constants::pi, +2.0 * prx::constants::pi });

    control_memory = { &_u[bike_dynamics_t::indices::thrust], &_u[bike_dynamics_t::indices::delta] };
    input_control_space = new space_t("EE", control_memory, "Torque");
    // input_control_space->set_bounds({ -0.6371781908344007 }, { 0.6371781908344007 });

    derivative_memory = { &_xd[0], &_xd[1], &_xd[2], &_xdd[0], &_xdd[1], &_xdd[2] };
    derivative_space = new space_t("EEEEEE", derivative_memory, "dynamic_vehicle_deriv");

    geometries["body"] = std::make_shared<prx::geometry_t>(prx::geometry_type_t::BOX);
    geometries["body"]->initialize_geometry({ 0.42, 0.25, 0.25 });
    geometries["body"]->generate_collision_geometry();
    geometries["body"]->set_visualization_color("0x00ff00");
    configurations["body"] = std::make_shared<prx::transform_t>();
    configurations["body"]->setIdentity();

    parameter_memory = { &_params[0], &_params[1], &_params[2], &_params[3],
                         &_params[4], &_params[5], &_params[6], &_params[7] };
    parameter_space = new space_t("EEEEEEEE", parameter_memory, "params");
  }

  virtual ~dynamic_vehicle_t()
  {
  }

  virtual void propagate(const double simulation_step) override final
  {
    // prx_assert(_model_initialized, "Torch model has not been initialized");
    // _xdd = dynamic_bycicle_factors_t::acceleration();
    _xdd = BikeDynamics::acceleration(_xd, _u, _params, _static_params);
    // PRX_DBG_VARS(_xdd.transpose());
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
    if (params.exists("static_params"))
    {
      std::vector<double> static_params_in{ params["static_params"].as<std::vector<double>>() };
      prx_assert(static_params_in.size() == BikeDynamics::DimStaticParams,
                 "[dynamic_vehicle] static params mismatch size");
      for (int i = 0; i < static_params_in.size(); ++i)
      {
        _static_params[i] = static_params_in[i];
      }
    }
    if (params.exists("params"))
    {
      std::vector<double> params_in{ params["params"].as<std::vector<double>>() };
      prx_assert(params_in.size() == BikeDynamics::DimParams, "[dynamic_vehicle] params mismatch size");
      for (int i = 0; i < params_in.size(); ++i)
      {
        _params[i] = params_in[i];
      }
    }
  }

  static prx::param_loader init()
  {
    prx::param_loader params{ prx::plant_t::init() };
    params["torch_file"].set("path_to_file");
    params["static_params"].set(std::vector<double>({ 3.5, 0.155, 0.155 }));

    std::vector<double> bike_params(BikeDynamics::DimParams, 1.0);
    params["params"].set(bike_params);
    return params;
  }

protected:
  State _x;
  Velocity _xd;
  Acceleration _xdd;
  Control _u;
  Params _params;
  StaticParams _static_params;

  TorchInterface _nn;
  bool _model_initialized;
};
}  // namespace fg
}  // namespace prx

PRX_REGISTER_SYSTEM(fg::dynamic_vehicle_t, dynamic_vehicle)
#endif