#pragma once

#include <memory>
#include "general/param_loader.hpp"
#include "prx/utilities/defs.hpp"
#include "prx/utilities/spaces/space_v2.hpp"
#include "prx/simulation/dynamical_system.hpp"
#include <gtsam/geometry/Pose2.h>

namespace prx
{

// class car_t : prx::dynamical_system_t<car_t>
// {
// public:
//   inline constexpr std::string = "Car";
//   using State = gtsam::ProductLieGroup<gtsam::Pose2, Eigen::Vector3d>;
//   using Control = Eigen::Vector2d;
//   using Parameters = Eigen::Vector2d;
//   using Observation = Eigen::Vector3d;

//   using StateSpace = prx::experimental::space_t<State>;
//   using ControlSpace = prx::experimental::space_t<Control>;
//   using ParametersSpace = prx::experimental::space_t<Parameters>;
//   using ObservationSpace = prx::experimental::space_t<Observation>;

//   using StateSpacePtr = std::shared_ptr<StateSpace>;
//   using ControlSpacePtr = std::shared_ptr<ControlSpace>;
//   using ParametersSpacePtr = std::shared_ptr<ParametersSpace>;
//   using ObservationSpacePtr = std::shared_ptr<ObservationSpace>;

//   void initialize()
//   {
//     prx::param_loader params;
//     const std::string state_space_bounds_yaml =
//         "bounds:\n"
//         "  -\n"
//         "    min: [-10, -10, -3.14159]\n"
//         "    max: [+10, +10, +3.14159]\n"
//         "  -\n"
//         "    min: [-0.5, -0.5, -0.1]\n"
//         "    max: [+0.5, +0.5, +0.1]\n";
//     const std::string control_space_bounds_yaml =
//         "bounds:\n"
//         "    min: [-1, -1]\n"
//         "    max: [+1, +1]\n";
//     const std::string observation_space_bounds_yaml =
//         "bounds:\n"
//         "    min: [-10, -10, -3.14159]\n"
//         "    max: [+10, +10, +3.14159]\n";

//     params["state_space"].from_string(state_space_bounds_yaml);
//     params["control_space"].from_string(state_space_bounds_yaml);
//     params["observation_space"].from_string(state_space_bounds_yaml);

//     _state_space = StateSpace::create(params["state_space"]);
//     _control_space = ControlSpace::create(params["control_space"]);
//     _parameter_space = ParametersSpace::create();
//     _sensor_space = ObservationSpace::create(params["observation_space"]);
//   }

//   virtual ~car_t() {};

//   virtual void operator()(const State& x0, const Control& u0, const double& dt)
//   {
//   }
//   // virtual void sense(const State& x0, const Control& u0, const double& dt, const Parameters& params) = 0;

//   inline std::string name() const
//   {
//     return _name;
//   }

//   friend std::ostream& operator<<(std::ostream& os, const dynamical_system_t& obj)
//   {
//     // static_cast<DynamicalSystem>(obj)->to_stream(os);
//     return os;
//   }

// protected:
//   const std::string _name;

//   StateSpacePtr _state_space;
//   ControlSpacePtr _control_space;
//   ParametersSpacePtr _parameter_space;
//   ObservationSpacePtr _sensor_space;
// };
}  // namespace prx
