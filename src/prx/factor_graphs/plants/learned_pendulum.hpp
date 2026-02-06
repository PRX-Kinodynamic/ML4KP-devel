#ifndef TORCH_NOT_BUILT

#pragma once
#include <torch/script.h>  // One-stop header.

#include "prx/simulation/plant.hpp"
#include "prx/factor_graphs/factors/euler_integration_factor.hpp"
#include "prx/factor_graphs/lie_groups/lie_integrator.hpp"
#include "prx/utilities/general/torch_interface.hpp"
#include "prx/factor_graphs/lie_groups/so2.hpp"

namespace prx
{
namespace fg
{

class learned_pendulum_t : public plant_t
{
  using SO2 = prx::fg::SO2_t;
  using Config = Eigen::Vector<double, 1>;
  using Control = Eigen::Vector<double, 1>;
  using LieIntegrator = prx::fg::lie_integrator_t<SO2, Config, double>;
  using EulerIntegrator = prx::fg::euler_integration_factor_t<Config, Config, double>;

  using TorchModule = torch::jit::script::Module;
  using TorchModulePtr = std::shared_ptr<TorchModule>;
  using TorchInterface = prx::utilities::torch_nn_interface_t<TorchModule, Config, Config, Control>;

public:
  learned_pendulum_t(const std::string& path) : plant_t(path)
  {
    state_memory = { &_x[0], &_xd[0] };
    state_space = new space_t("RE", state_memory, "learned_pendulum_state");
    state_space->set_bounds({ -prx::constants::pi, -2.0 * prx::constants::pi },
                            { +prx::constants::pi, +2.0 * prx::constants::pi });

    control_memory = { &_u[0] };
    input_control_space = new space_t("E", control_memory, "Torque");
    input_control_space->set_bounds({ -0.6371781908344007 }, { 0.6371781908344007 });

    derivative_memory = { &_xd[0], &_xdd[0] };
    derivative_space = new space_t("EE", derivative_memory, "pendulum_deriv");

    const double length = 20;

    geometries["rod1"] = std::make_shared<geometry_t>(geometry_type_t::BOX);
    geometries["rod1"]->initialize_geometry({ length, 1, 1 });
    geometries["rod1"]->generate_collision_geometry();
    geometries["rod1"]->set_visualization_color("0x00ff00");
    configurations["rod1"] = std::make_shared<transform_t>();
    configurations["rod1"]->setIdentity();

    geometries["ball"] = std::make_shared<geometry_t>(geometry_type_t::SPHERE);
    geometries["ball"]->initialize_geometry({ 1.5 });
    geometries["ball"]->generate_collision_geometry();
    geometries["ball"]->set_visualization_color("0x0000ff");
    configurations["ball"] = std::make_shared<transform_t>();
    configurations["ball"]->setIdentity();
  }

  virtual ~learned_pendulum_t()
  {
  }

  virtual void propagate(const double simulation_step) override final
  {
    _xdd = _nn(_xd, _u);
    _xdd[0] += _gravity / _length * std::sin(_x[0]);

    _xd = EulerIntegrator::integrate(_xd, _xdd, simulation_step);
    _x = LieIntegrator::integrate(_x, _xd, simulation_step);
  }

  virtual void update_configuration() override
  {
    const double theta1 = _x[0] - M_PI / 2.0;

    const double length = 20;
    auto body = configurations["rod1"];
    body->setIdentity();
    Eigen::Matrix3d m;
    m = Eigen::AngleAxisd(-theta1, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
    body->linear() = (m);
    body->translation() = (vector_t((length / 2.0) * cos(theta1), -(length / 2.0) * sin(theta1), 1.5));

    body = configurations["ball"];
    body->setIdentity();
    body->translation() = (vector_t((length)*cos(theta1), -(length)*sin(theta1), 1.5));
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
      _nn.load_model(torch_filename);
    }
  }

  virtual prx::param_loader init() override
  {
    prx::param_loader params{ prx::plant_t::init() };
    params["torch_file"].set("");

    return params;
  }

protected:
  const double _gravity = 9.81;
  double _length = 0.5;
  // double friction = 0.1;
  // double inertia;
  // double mass = 0.15;
  // double normalize = true;

  SO2 _x;
  Config _xd;
  Config _xdd;
  Control _u;

  TorchInterface _nn;
};
}  // namespace fg
}  // namespace prx

PRX_REGISTER_SYSTEM(fg::learned_pendulum_t, learned_pendulum)
#endif