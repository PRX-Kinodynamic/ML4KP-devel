#pragma once

#include <memory>
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/utilities/defs.hpp"
#include "prx/utilities/spaces/space_v2.hpp"
#include "prx/utilities/geometry/geometry.hpp"

namespace prx
{

template <typename Derived>
struct dynamical_system_traits
{
  // clang-format off
  // enum {StateDimension = 6}; 
  // enum {ControlDimension = 3}; 
  // enum {ParametersDimension = 3}; 
  // enum {ObservationDimension = 3};
  // clang-format on

  using DerivedSystem = Derived;
  // using State = typename Derived::State;
  // using Control = typename Derived::Control;
  // using Parameters = typename Derived::Parameters;
  // using Observation = typename Derived::Observation;
};

// struct dynamical_system_base_t
// {
//   virtual std::string name() const = 0;

//   // virtual std::string cast() const = 0;
// };

template <typename Derived>
class dynamical_system_t  // : dynamical_system_base_t
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
    : _state_space(std::make_shared<StateSpace>(params["state_space"]))
    , _control_space(std::make_shared<ControlSpace>(params["control_space"]))
    , _parameter_space(std::make_shared<ParametersSpace>(params["parameter_space"]))
    , _sensor_space(std::make_shared<ObservationSpace>(params["sensor_space"]))
  {
  }

  dynamical_system_t()
    : _state_space(nullptr), _control_space(nullptr), _parameter_space(nullptr), _sensor_space(nullptr)
  {
    static_cast<Derived*>(this)->initialize();
  }

  // template <typename... Args>
  // static DerivedDynamicalSystemPtr create(Args... args)
  // {
  //   return std::make_shared<DerivedDynamicalSystem>(args...);
  // }

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

  Observation sense(const State& x0)
  {
    return static_cast<Derived*>(this)->sense(x0);
  }

  State propagate(const State& x0, const Control& u0, const double& dt)
  {
    return static_cast<Derived*>(this)->propagate(x0, u0, dt);
  }

  std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> configuration(const State& state)
  {
    return static_cast<Derived*>(this)->configuration(state);
  }

  std::vector<std::shared_ptr<prx::geometry_t>> geometries()
  {
    if (not _geometries_initialized)
    {
      static_cast<Derived*>(this)->initialize_geometries();
      _geometries_initialized = true;
    }

    return _geometries;
  }

  void environment(const prx::obstacle_loader_t& loader)
  {
    return static_cast<Derived*>(this)->environment(loader);
  }

  inline std::string name() const
  {
    return Derived::Name;
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
  // Parameters _parameters;

  StateSpacePtr _state_space;
  ControlSpacePtr _control_space;
  ParametersSpacePtr _parameter_space;
  ObservationSpacePtr _sensor_space;

  bool _geometries_initialized;
  std::vector<std::shared_ptr<prx::geometry_t>> _geometries;
};
}  // namespace prx
