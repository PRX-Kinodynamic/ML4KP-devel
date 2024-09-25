#pragma once

#include "prx/simulation/plant.hpp"
#include "prx/factor_graphs/factors/pushing_contact_factors.hpp"
#include "prx/factor_graphs/factors/euler_integration_factor.hpp"

namespace prx
{
namespace fg
{

class pusher_slider_t : public plant_t
{
  using BodyVelocity = prx::fg::pushing_types::BodyVelocity;
  using Pose = prx::fg::pushing_types::Pose;
  using Position = prx::fg::pushing_types::Position;
  using Parameters = pushing_types::Parameters::Values;

  using ContactVelocity = prx::fg::pushing_types::ContactVelocity;
  using LieIntegrator = prx::fg::lie_integration_factor_t<Pose, BodyVelocity>;
  using StickingContact = prx::fg::sticking_contact_t;
  using UpContact = prx::fg::sliding_pushing_contact_t<true>;
  using DownContact = prx::fg::sliding_pushing_contact_t<false>;
  using SlippingContactVel = prx::fg::sliping_contact_velocity_t;
  using EulerIntegrator = prx::fg::euler_integration_factor_t<Position, ContactVelocity>;

public:
  pusher_slider_t(const std::string& path)
    : plant_t(path)
    , _x(Pose::Zero())
    , _xdot(BodyVelocity::Zero())
    , _u(ContactVelocity::Zero())
    , _contact_position(Position::Zero())
    , _params(Parameters::Zero())
  {
    state_memory = { &_x[0], &_x[1], &_x[2], &_contact_position[0], &_contact_position[1] };
    state_space = new space_t("EEEEE", state_memory, "pusher_slider_state");

    control_memory = { &_u[0], &_u[1] };
    input_control_space = new space_t("EE", control_memory, "pusher_slider_control");

    parameter_memory = { &_params[0], &_params[1] };
    parameter_space = new space_t("EE", parameter_memory, "pusher_slider_params");

    geometries["body"] = std::make_shared<geometry_t>(geometry_type_t::BOX);
    geometries["body"]->initialize_geometry({ 1, 1, 1 });
    geometries["body"]->generate_collision_geometry();
    geometries["body"]->set_visualization_color("0x00ff00");
    configurations["body"] = std::make_shared<transform_t>();
    configurations["body"]->setIdentity();
  }

  virtual ~pusher_slider_t()
  {
  }

  virtual void propagate(const double simulation_step) override final
  {
    if (StickingContact::active(_contact_position, _u, _params))
    {
      _xdot = StickingContact::predict(_contact_position, _u, _params);
      _vb = _u;
    }
    else if (UpContact::active(_contact_position, _u, _params))
    {
      _vb = UpContact::predict(_contact_position, _u, _params);
      _xdot = StickingContact::predict(_contact_position, _vb, _params);
    }
    else if (DownContact::active(_contact_position, _u, _params))
    {
      _vb = DownContact::predict(_contact_position, _u, _params);
      _xdot = StickingContact::predict(_contact_position, _vb, _params);
    }

    _vslip = SlippingContactVel::predict(_vb, _u);
    _contact_position = EulerIntegrator::predict(_contact_position, _vslip, simulation_step);

    _x = LieIntegrator::predict(_x, _xdot, simulation_step);
  }

  virtual void update_configuration() override
  {
    auto body = configurations["body"];
    body->linear() = Eigen::Matrix3d::Identity();
    body->linear().block<2, 2>(0, 0) = _x.rotation<Eigen::Matrix2d>();
    body->translation().head(2) = _x.translation();
    body->translation()[2] = 0.5;  // z is fix
  }

  virtual void compute_derivative() override final
  {
  }

protected:
  Pose _x;
  BodyVelocity _xdot;
  ContactVelocity _u;
  Position _contact_position;
  Parameters _params;

  ContactVelocity _vb, _vslip;
};
}  // namespace fg
}  // namespace prx

PRX_REGISTER_SYSTEM(prx::fg::pusher_slider_t, pusher_slider)
