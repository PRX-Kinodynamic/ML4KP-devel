#pragma once

#include <prx/utilities/general/param_loader.hpp>
#include <prx/utilities/spaces/space_snapshot.hpp>
#include <string>
#include <memory>

// ML4KP
#include "prx/factor_graphs/factors/euler_integration_factor.hpp"
#include "prx/factor_graphs/utilities/symbols_factory.hpp"
#include "prx/factor_graphs/factors/quadratic_cost_factor.hpp"
#include "prx/factor_graphs/lie_groups/lie_integrator.hpp"
#include "prx/simulation/dynamical_system.hpp"
#include "prx/utilities/spaces/space_v2.hpp"

// Gtsam
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/geometry/Pose2.h>

namespace prx
{

class dubins_evader_pursuit_t;

template <>
struct dynamical_system_traits<dubins_evader_pursuit_t>
{
  // clang-format off
  enum {StateDimension = 3}; 
  enum {ControlDimension = 2}; 
  enum {ParametersDimension = 1}; 
  enum {ObservationDimension = 3};
  // clang-format on

  using State = gtsam::Pose2;
  using Control = Eigen::Vector2d;
  using Parameters = Eigen::Vector<double, ParametersDimension>;
  using Observation = State;

  using StateDot = Eigen::Vector<double, StateDimension>;
};

class dubins_evader_pursuit_t : public prx::dynamical_system_t<dubins_evader_pursuit_t>
{
public:
  using Base = prx::dynamical_system_t<dubins_evader_pursuit_t>;
  using Derived = dubins_evader_pursuit_t;

  const std::string Name = "DubinsEvaderPursuit";
  using State = typename Base::State;
  using Control = typename Base::Control;
  using Parameters = typename Base::Parameters;
  using Observation = typename Base::Observation;

  using StateSpace = typename Base::StateSpace;
  using ControlSpace = typename Base::ControlSpace;
  using ParametersSpace = typename Base::ParametersSpace;
  using ObservationSpace = typename Base::ObservationSpace;

  using StateSpacePtr = std::shared_ptr<StateSpace>;
  using ControlSpacePtr = std::shared_ptr<ControlSpace>;
  using ParametersSpacePtr = std::shared_ptr<ParametersSpace>;
  using ObservationSpacePtr = std::shared_ptr<ObservationSpace>;

  dubins_evader_pursuit_t(prx::param_loader params) : Base(params), _vp({ 0.75, 0. }), _ve({ 0.75, 0. })
  {
  }
  dubins_evader_pursuit_t() : dubins_evader_pursuit_t(default_params()) {};

  dubins_evader_pursuit_t(const std::string params) : dubins_evader_pursuit_t(prx::param_loader::create(params))
  {
  }

  static prx::param_loader default_params()
  {
    prx::param_loader params;
    const std::string state_space_bounds_yaml =
        "bounds:\n"
        "  -\n"
        "    min: [-1.0, -1.0, -3.14159]\n"
        "    max: [+1.0, +1.0, +3.14159]\n";
    const std::string control_space_bounds_yaml =
        "bounds:\n"
        "    min: [-3, -3]\n"
        "    max: [+3, +3]\n";
    const std::string observation_space_bounds_yaml{ state_space_bounds_yaml };

    params["state_space"].from_string(state_space_bounds_yaml);
    params["control_space"].from_string(control_space_bounds_yaml);
    params["observation_space"].from_string(observation_space_bounds_yaml);

    return params;
  }

  void initialize_geometries()
  {
    _geometries.push_back(std::make_shared<prx::geometry_t>(prx::geometry_type_t::BOX));
    _geometries.back()->initialize_geometry({ 0.1, 0.05, 0.05 });
    _geometries.back()->generate_collision_geometry();
    _geometries.back()->set_visualization_color("0x00ff00");
  }

  void initialize()
  {
    prx::param_loader params{ default_params() };

    _state_space = StateSpace::create(params["state_space"]);
    _control_space = ControlSpace::create(params["control_space"]);
    _sensor_space = ObservationSpace::create(params["observation_space"]);
    _parameter_space = ParametersSpace::create(params["parameter_space"]);
  }

  // virtual ~dubins_evader_pursuit_t() {};

  Observation sense(const State& x)
  {
    return x;
  }

  State propagate(const State& x0, const Control& u0, const double& dt,  // no-lint
                  OptJacX Hx = nullptr, OptJacU Hu = nullptr, OptJacDT Hdt = nullptr)
  {
    using LieIntegrator = prx::fg::lie_integrator_t<State, Eigen::Vector3d>;
    const bool compute_deriv{ Hx or Hu or Hdt };

    // The system is:
    // xdot =  [-ve + vp * cos(th) + we * y]
    // ydot =  [      vp * sin(th) - we * x]
    // thdot = [wp - we]
    //
    // This can be rewritten (for getting the derivatives) as:
    // xydot = [R(th) * [vp; 0] + R(-pi/2)*[x;y] - [ve; 0] ]
    // thdot = [wp - we]
    //
    // Where R(.) is the rotation matrix.
    // Could be further simplify using the Adjoint: Ad(x,y,th) = [R(th) [y;-x];0 0 1]
    // xdot = Ad(x,y,th)*[vp 0 we]' - [ve 0 -wp+2*we]'
    // But gtsam does not provide derivatives for Adjoint

    const double& wp{ u0[0] };
    const double& we{ u0[1] };
    // Eigen::Matrix<double, 2, 3> adjXY_H_x0{ Eigen::Matrix2d::Identity() };

    const Eigen::Vector2d adj_xy{ { x0.y(), -x0.x() } };
    const Eigen::Vector2d linear_xdot{ x0.transformFrom(_vp) + we * adj_xy - _ve };
    const double angular_xdot{ wp - we };

    // const Eigen::Matrix2d weAdj_H_xy{ (Eigen::Matrix2d() << 0., we, -we, 0.).finished() };
    // const Eigen::Vector2d& weAdj_H_we{ adj_xy };
    // const Eigen::Matrix2d lxdot_H_x0XY{ adjXY_H_x0 + weAdj_H_xy };
    // const Eigen::Matrix<double, 1, 1> angxdot_H_x0XY{ adjXY_H_x0 + weAdj_H_xy };

    // Eigen::Matrix3d xdot_H_{ Eigen::Matrix3d::Identity() };
    const Eigen::Vector3d xdot{ (Eigen::Vector3d() << linear_xdot, angular_xdot).finished() };

    Eigen::Matrix3d x1_H_x0{ Eigen::Matrix3d::Identity() };
    Eigen::Matrix3d x1_H_xdot{ Eigen::Matrix3d::Identity() };
    const State x1{ LieIntegrator::integrate(x0, xdot, dt,                        // no-lint
                                             compute_deriv ? &x1_H_x0 : nullptr,  // no-lint
                                             compute_deriv ? &x1_H_xdot : nullptr) };

    if (Hx)
    {
      *Hx = x1_H_x0;
    }
    if (Hu)
    {
      prx_warn("[dubins_evader_pursuit_t::propagate] Hu derivative not implemented")
      // *Hu = x1_H_dot*
    }
    if (Hdt)
    {
      prx_warn("[dubins_evader_pursuit_t::propagate] Hdt derivative not implemented")
    }

    return x1;
  }

  std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> configuration(const State& state)
  {
    const Eigen::Matrix3d R{ prx::axis_to_rotation_matrix({ state.theta() }, 'Z') };
    const Eigen::Vector3d t(state.x(), state.y(), 0.0);
    return { { R, t } };
  }

  void environment(const prx::obstacle_loader_t& loader)
  {
    auto bounds = _state_space->sampler.bounds();
    auto env_min_bounds = loader.min_bounds();
    auto env_max_bounds = loader.max_bounds();
    bounds.first = env_min_bounds;
    bounds.second = env_max_bounds;
    _state_space->sampler.bounds(bounds.first, bounds.second);
  }

protected:
  const Eigen::Vector2d _vp;
  const Eigen::Vector2d _ve;
};

};  // namespace prx