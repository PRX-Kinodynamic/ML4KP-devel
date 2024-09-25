#pragma once

#include "prx/factor_graphs/lie_groups/se2.hpp"
#include "prx/factor_graphs/lie_groups/lie_integrator.hpp"
#include "prx/factor_graphs/factors/euler_integration_factor.hpp"
#include "prx/simulation/plant.hpp"

namespace prx
{
namespace fg
{

template <uint8_t Order>
class SE2_rigid_body_t : public plant_t
{
  using SE2 = prx::fg::SE2_t;
  using Velocity = Eigen::Vector<double, 3>;
  using Acceleration = Eigen::Vector<double, 3>;
  using LieIntegrator = prx::fg::lie_integrator_t<SE2, Velocity>;
  using EulerIntegrator = prx::fg::euler_integration_factor_t<Velocity, Acceleration>;

public:
  template <Eigen::Index OrderIn = Order, std::enable_if_t<(OrderIn == 1), bool> = true>
  SE2_rigid_body_t(const std::string& path) : plant_t(path), _pose()
  {
    state_memory = { &_pose[0], &_pose[1], &_pose[2] };
    state_space = new space_t("EEE", state_memory, "SE2State1stOrder");

    derivative_memory = { &_velocity[0], &_velocity[1], &_velocity[2] };
    derivative_space = new space_t("EEE", derivative_memory, "SE2Deriv1stOrder");

    control_memory = { &_velocity[0], &_velocity[1], &_velocity[2] };
    input_control_space = new space_t("EEE", control_memory, "SE2Control1stOrder");
    init_geometry();
  }

  template <Eigen::Index OrderIn = Order, std::enable_if_t<(OrderIn == 2), bool> = true>
  SE2_rigid_body_t(const std::string& path) : plant_t(path), _pose()
  {
    state_memory = { &_pose[0], &_pose[1], &_pose[2], &_velocity[0], &_velocity[1], &_velocity[2] };
    state_space = new space_t("EEEEEE", state_memory, "SE2State2ndOrder");

    derivative_memory = { &_velocity[0],     &_velocity[1],     &_velocity[2],
                          &_acceleration[0], &_acceleration[1], &_acceleration[2] };
    derivative_space = new space_t("EEEEEE", derivative_memory, "SE2Deriv2ndOrder");

    control_memory = { &_acceleration[0], &_acceleration[1], &_acceleration[2] };
    input_control_space = new space_t("EEE", control_memory, "SE2Control2ndOrder");
    init_geometry();
  }

  virtual ~SE2_rigid_body_t()
  {
  }

  virtual void propagate(const double simulation_step) override final
  {
    if constexpr (Order == 2)
    {
      _velocity = EulerIntegrator::integrate(_velocity, _acceleration, simulation_step);
    }
    _pose = LieIntegrator::integrate(_pose, _velocity, simulation_step);
  }

  virtual void update_configuration() override
  {
    auto body = configurations["body"];
    body->translation().head(2) = _pose.translation();
    body->linear().template block<2, 2>(0, 0) = _pose.rotation<Eigen::Matrix2d>();
  }

  virtual void compute_derivative() override
  {
  }

protected:
  void init_geometry()
  {
    geometries["body"] = std::make_shared<geometry_t>(geometry_type_t::BOX);
    geometries["body"]->initialize_geometry({ 1.618, 1.0, 1.0 });
    geometries["body"]->generate_collision_geometry();
    geometries["body"]->set_visualization_color("0x00ff00");
    configurations["body"] = std::make_shared<transform_t>();
    configurations["body"]->setIdentity();
  }

  SE2 _pose;
  Velocity _velocity;
  Acceleration _acceleration;
};
}  // namespace fg
}  // namespace prx

PRX_REGISTER_SYSTEM(prx::fg::SE2_rigid_body_t<1>, SE2_rigid_body_1st_order)
PRX_REGISTER_SYSTEM(prx::fg::SE2_rigid_body_t<2>, SE2_rigid_body_2nd_order)
