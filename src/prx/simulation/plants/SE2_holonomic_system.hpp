#ifdef PRX_GTSAM_AVAILABLE

#pragma once

#include <memory>
#include "prx/utilities/general/param_loader.hpp"
#include "prx/utilities/defs.hpp"
#include <gtsam/geometry/Pose2.h>
#include "prx/utilities/spaces/space_v2.hpp"
#include "prx/simulation/dynamical_system.hpp"

namespace prx
{
class SE2_holonomic_system_t;
template <>
struct dynamical_system_traits<SE2_holonomic_system_t>
{
  // clang-format off
  enum {StateDimension = 6}; 
  enum {ControlDimension = 3}; 
  enum {ParametersDimension = 3}; 
  enum {ObservationDimension = 3};
  // clang-format on

  using State = gtsam::ProductLieGroupV43<gtsam::Pose2, Eigen::Vector3d>;
  using Control = Eigen::Vector<double, ControlDimension>;
  using Parameters = Eigen::Vector<double, ParametersDimension>;
  using Observation = gtsam::Pose2;

  using StateDot = Eigen::Vector<double, StateDimension>;
};

class SE2_holonomic_system_t : public prx::dynamical_system_t<SE2_holonomic_system_t>
{
public:
  const std::string Name = "SE2HolonomicSystem";
  using Base = prx::dynamical_system_t<SE2_holonomic_system_t>;
  using Derived = SE2_holonomic_system_t;

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

  SE2_holonomic_system_t() : Base() {};
  SE2_holonomic_system_t(prx::param_loader params) : Base(params) {};

  static prx::param_loader default_params()
  {
    PRX_DEBUG_PRINT
    prx::param_loader params;
    const std::string state_space_bounds_yaml =
        "bounds:\n"
        "  -\n"
        "    min: [-10, -10, -3.14159]\n"
        "    max: [+10, +10, +3.14159]\n"
        "  -\n"
        "    min: [-0.5, -0.5, -0.1]\n"
        "    max: [+0.5, +0.5, +0.1]\n";
    const std::string control_space_bounds_yaml =
        "bounds:\n"
        "    min: [-1.0, -1.0, -1.0]\n"
        "    max: [+1.0, +1.0, +1.0]\n";
    const std::string observation_space_bounds_yaml =
        "bounds:\n"
        "    min: [-10, -10, -3.14159]\n"
        "    max: [+10, +10, +3.14159]\n";
    const std::string parameter_space_bounds_yaml =
        "bounds:\n"
        "    min: [-1, -1, -1]\n"
        "    max: [+1, +1, +1]\n";

    // PRX_DEBUG_PRINT
    // params.print();

    // params["test"].set("t");
    // PRX_DBG_VARS(params)
    // PRX_DBG_VARS(state_space_bounds_yaml)
    // params["state_space"].set("state");
    // PRX_DBG_VARS(params)
    params["state_space"].from_string(state_space_bounds_yaml);
    PRX_DEBUG_PRINT
    // PRX_DBG_VARS(params)
    params["control_space"].from_string(control_space_bounds_yaml);
    PRX_DEBUG_PRINT
    // params.print();
    params["observation_space"].from_string(observation_space_bounds_yaml);
    // PRX_DEBUG_PRINT
    // params.print();
    params["parameter_space"].from_string(parameter_space_bounds_yaml);
    PRX_DEBUG_PRINT
    // params.print();
    PRX_DBG_VARS(params)

    PRX_DEBUG_PRINT
    return params;
  }

  void initialize_geometries()
  {
    _geometries.push_back(std::make_shared<geometry_t>(geometry_type_t::SPHERE));
    _geometries.back()->initialize_geometry({ 0.5 });
    _geometries.back()->generate_collision_geometry();
    _geometries.back()->set_visualization_color("0x00ff00");
  }

  void initialize()
  {
    PRX_DEBUG_PRINT
    prx::param_loader params{ default_params() };
    PRX_DEBUG_PRINT
    PRX_DBG_VARS(params)

    // params.print();
    PRX_DEBUG_PRINT
    auto state = params["state_space"];
    PRX_DBG_VARS(state)
    PRX_DBG_VARS(state["bounds"])
    _state_space = StateSpace::create(state);
    PRX_DEBUG_PRINT
    _control_space = ControlSpace::create(params["control_space"]);
    PRX_DEBUG_PRINT
    _sensor_space = ObservationSpace::create(params["observation_space"]);
    PRX_DEBUG_PRINT
    _parameter_space = ParametersSpace::create(params["parameter_space"]);
    PRX_DEBUG_PRINT
  }

  virtual ~SE2_holonomic_system_t() {};

  static double distance(const State& a, const State& b)
  {
    using ErrorVector = Eigen::Vector<double, dynamical_system_traits<SE2_holonomic_system_t>::StateDimension>;
    const State between{ a.between(b) };
    const ErrorVector error{ State::Logmap(between) };
    return error.norm();
  }

  Observation sense(const State& x0)
  {
    return x0.first;
  }

  State propagate(const State& x0, const Control& u0, const double& dt, OptJacX Hx = nullptr, OptJacU Hu = nullptr,
                  OptJacDT Hdt = nullptr)
  {
    Eigen::Vector<double, 6> xdot;
    xdot << x0.second, u0 * dt;
    const State x01{ State::Expmap(xdot) };
    const State x1{ gtsam::traits<State>::Compose(x0, x01) };
    if (Hx)
    {
      prx_warn("Jacobians are wrong, used only for testing compilation");
      *Hx = std::remove_pointer_t<OptJacX>::Identity();
    }
    if (Hu)
    {
      *Hu = std::remove_pointer_t<OptJacU>::Identity();
    }
    if (Hdt)
    {
      *Hdt = std::remove_pointer_t<OptJacDT>::Identity();
    }
    return x1;
  }

  std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> configuration(const State& state)
  {
    const Eigen::Matrix3d R{ prx::axis_to_rotation_matrix({ state.first.theta() }, 'Z') };
    const Eigen::Vector3d t(state.first.x(), state.first.y(), 0.0);
    return { { R, t } };
  }

  void environment(const prx::obstacle_loader_t& loader)
  {
    auto bounds = _state_space->sampler.bounds();
    auto env_min_bounds = loader.min_bounds();
    auto env_max_bounds = loader.max_bounds();
    bounds.first.first.head(2) = env_min_bounds.head(2);
    bounds.first.second.head(2) = env_max_bounds.head(2);
    _state_space->sampler.bounds(bounds.first.first, bounds.first.second, bounds.second.first, bounds.second.second);
  }
  // virtual void sense(const State& x0, const Control& u0, const double& dt, const Parameters& params) = 0;

protected:
  StateSpacePtr _state_space;
  ControlSpacePtr _control_space;
  ParametersSpacePtr _parameter_space;
  ObservationSpacePtr _sensor_space;
};

}  // namespace prx

#endif