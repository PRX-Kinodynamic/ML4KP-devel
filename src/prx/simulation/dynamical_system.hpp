#pragma once

#include <memory>
#include "general/param_loader.hpp"
#include "loaders/obstacle_loader.hpp"
#include "prx/utilities/defs.hpp"
#include "prx/utilities/spaces/space_v2.hpp"
#include "prx/utilities/geometry/geometry.hpp"

namespace prx
{

template <typename Derived>
struct dynamical_system_traits
{
  // using State = typename Derived::State;
  // using Control = typename Derived::Control;
  // using Parameters = typename Derived::Parameters;
  // using Observation = typename Derived::Observation;
};
// {
// }

// template <typename StateSpace, typename ControlSpace, typename ParameterSpace, typename SensorSpace>
template <typename Derived>
class dynamical_system_t
{
public:
  using DerivedDynamicalSystem = dynamical_system_t<Derived>;
  using DerivedDynamicalSystemPtr = std::shared_ptr<DerivedDynamicalSystem>;

  using State = typename dynamical_system_traits<Derived>::State;
  using Control = typename dynamical_system_traits<Derived>::Control;
  using Parameters = typename dynamical_system_traits<Derived>::Parameters;
  using Observation = typename dynamical_system_traits<Derived>::Observation;

  using StateSpace = prx::experimental::space_t<State>;
  using ControlSpace = prx::experimental::space_t<Control>;
  using ParametersSpace = prx::experimental::space_t<Parameters>;
  using ObservationSpace = prx::experimental::space_t<Observation>;

  using StateSpacePtr = std::shared_ptr<StateSpace>;
  using ControlSpacePtr = std::shared_ptr<ControlSpace>;
  using ParametersSpacePtr = std::shared_ptr<ParametersSpace>;
  using ObservationSpacePtr = std::shared_ptr<ObservationSpace>;

  dynamical_system_t(prx::param_loader params)
    : _name(params["name"].as<>())
    , _state_space(std::make_shared<StateSpace>(params["state_space"]))
    , _control_space(std::make_shared<ControlSpace>(params["control_space"]))
    , _parameter_space(std::make_shared<ParametersSpace>(params["parameter_space"]))
    , _sensor_space(std::make_shared<ObservationSpace>(params["sensor_space"]))
  {
  }

  dynamical_system_t(const std::string name)
    : _name(name), _state_space(nullptr), _control_space(nullptr), _parameter_space(nullptr), _sensor_space(nullptr)
  {
    static_cast<Derived*>(this)->initialize();
  }

  static double distance(const State& a, const State& b)
  {
    return Derived::distance(a, b);
  }

  static prx::param_loader default_params()
  {
    return DerivedDynamicalSystem::default_params();
  }
  virtual ~dynamical_system_t() {};

  inline StateSpacePtr state_space() const
  {
    return _state_space;
  }
  inline ControlSpacePtr control_space() const
  {
    return _control_space;
  }
  inline ParametersSpacePtr parameter_space() const
  {
    return _parameter_space;
  }
  inline ObservationSpacePtr sensor_space() const
  {
    return _sensor_space;
  }

  State propagate(const State& x0, const Control& u0, const double& dt)
  {
    return static_cast<Derived*>(this)->propagate(x0, u0, dt);
  }
  // virtual void sense(const State& x0, const Control& u0, const double& dt, const Parameters& params) = 0;

  std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> configuration(const State& state)
  {
    return static_cast<Derived*>(this)->configuration(state);
  }

  std::vector<std::shared_ptr<prx::geometry_t>> geometries()
  {
    return _geometries;
  }

  void environment(const prx::obstacle_loader_t& loader)
  {
    return static_cast<Derived*>(this)->environment(loader);
  }

  inline std::string name() const
  {
    return _name;
  }

  friend std::ostream& operator<<(std::ostream& os, const DerivedDynamicalSystem& obj)
  {
    static_cast<DerivedDynamicalSystem>(obj)->to_stream(os);
    return os;
  }

  friend std::ostream& operator<<(std::ostream& os, const DerivedDynamicalSystemPtr& obj)
  {
    os << (*obj);
    return os;
  }

protected:
  const std::string _name;

  // Parameters _parameters;

  StateSpacePtr _state_space;
  ControlSpacePtr _control_space;
  ParametersSpacePtr _parameter_space;
  ObservationSpacePtr _sensor_space;

  std::vector<std::shared_ptr<prx::geometry_t>> _geometries;
};
}  // namespace prx
