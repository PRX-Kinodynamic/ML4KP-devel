#pragma once
#include <array>
#include <numeric>
#include <functional>

#include <gtsam/config.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "prx/factor_graphs/utilities/symbols_factory.hpp"
#include "prx/factor_graphs/lie_groups/se2.hpp"

// Most of the equations are taken from [1] and adapted for FG-form. Friction cone angle from [2], constraints from [3]
// 			[1]	Lynch, Kevin M., Hitoshi Maekawa, and Kazuo Tanie. "Manipulation and active
// 					sensing by pushing using tactile feedback." In IROS, vol. 1, pp. 416-421. 1992.
// 			[2] Mason, Matthew T. "Mechanics and planning of manipulator pushing operations." The International Journal of
// 					Robotics Research 5, no. 3 (1986): 53-71.
// 			[3] Hogan, François Robert, and Alberto Rodriguez. "Feedback control of the pusher-slider system: A story of
// 					hybrid and underactuated contact dynamics." In Algorithmic Foundations of Robotics XII: Proceedings of the
// 					Twelfth Workshop on the Algorithmic Foundations of Robotics, pp. 800-815. Springer International Publishing,
// 					2020.

namespace prx
{
namespace fg
{
namespace pushing_types
{
using ContactVelocity = Eigen::Vector2d;
using BodyVelocity = Eigen::Vector3d;
using Position = Eigen::Vector2d;
using Pose = SE2_t;

namespace Parameters
{
const unsigned Dim{ 2 };
using Values = Eigen::Vector<double, Dim>;
const unsigned C_idx{ 0 };
const unsigned mu_idx{ 1 };

double gamma_t(const Values& vals, const Position& p)
{
  const double& c2{ std::pow(vals[pushing_types::Parameters::C_idx], 2) };
  const double& mu{ vals[pushing_types::Parameters::mu_idx] };
  const double& px{ p[0] };
  const double& py{ p[1] };

  const double pxpy{ px * py };

  return (mu * c2 - pxpy + mu * px * px) / (c2 + py * py - mu * pxpy);
}

double gamma_b(const Values& vals, const Position& p)
{
  const double& c2{ std::pow(vals[pushing_types::Parameters::C_idx], 2) };
  const double& mu{ vals[pushing_types::Parameters::mu_idx] };
  const double& px{ p[0] };
  const double& py{ p[1] };

  const double pxpy{ px * py };

  return (-mu * c2 - pxpy - mu * px * px) / (c2 + py * py + mu * pxpy);
}

}  // namespace Parameters
}  // namespace pushing_types

class sticking_contact_t : public gtsam::NoiseModelFactor3<pushing_types::BodyVelocity, pushing_types::Position,
                                                           pushing_types::ContactVelocity>
{
  using Base =
      gtsam::NoiseModelFactor3<pushing_types::BodyVelocity, pushing_types::Position, pushing_types::ContactVelocity>;
  using Derived = sticking_contact_t;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

public:
  using BodyVelocity = pushing_types::BodyVelocity;
  using Position = pushing_types::Position;
  using ContactVelocity = pushing_types::ContactVelocity;
  using Params = pushing_types::Parameters::Values;

  using Jacobian = Eigen::Matrix<double, 3, 3>;
  using OptDeriv = boost::optional<Eigen::MatrixXd&>;
  using OptJacobian = gtsam::OptionalJacobian<3, 3>;

  sticking_contact_t(const gtsam::Key key_body_vel, const gtsam::Key key_contact_position,
                     const gtsam::Key key_contact_velocity, const Params params, const NoiseModel& cost_model)
    : Base(cost_model, key_body_vel, key_contact_position, key_contact_velocity), _params(params)
  {
  }

  static BodyVelocity predict(const Position& position, const ContactVelocity& contact_velocity, const Params& params,
                              OptJacobian Hpos = boost::none, OptJacobian Hvel = boost::none)
  {
    const double c2{ std::pow(params[pushing_types::Parameters::C_idx], 2) };
    const double den{ 1.0 / c2 + position.squaredNorm() };  // 1.0/(c^2 + x^2 + y^2)
    BodyVelocity vb{ BodyVelocity::Zero() };

    const Eigen::Matrix2d I{ Eigen::Matrix2d::Identity() };
    const Eigen::Matrix2d c2I{ c2 * I };
    const Eigen::Matrix2d ppT{ position * position.transpose() };

    vb.head(2) = den * (c2I + ppT) * contact_velocity;
    vb[2] = (position[0] * vb[1] - position[1] * vb[0]) / c2;  // \omega

    if (Hpos)
    {
      *Hpos = Jacobian::Zero();
      (*Hpos).block<2, 2>(0, 0) = den * position.transpose() * contact_velocity * I +  // no-lint
                                  den * position * contact_velocity.transpose();
      (*Hpos)(2, 0) = vb[1] / c2;
      (*Hpos)(2, 1) = -vb[0] / c2;
    };
    if (Hvel)
    {
      *Hvel = Jacobian::Zero();
      (*Hvel).block<2, 2>(0, 0) = den * (c2I + ppT);
      (*Hvel)(2, 0) = -position[1] / c2;
      (*Hvel)(2, 1) = position[0] / c2;
    };

    return vb;
  }

  virtual bool active(const gtsam::Values& values) const override
  {
    const Position position{ this->template key<2>() };
    const ContactVelocity contact_velocity{ this->template key<3>() };
    return active(position, contact_velocity, _params);
  }

  static bool active(const Position& position, const ContactVelocity& contact_velocity, const Params& params)
  {
    const double& vn{ contact_velocity[0] };  // vx, normal
    const double& vt{ contact_velocity[1] };  // vy, tangential

    const double gamma_b{ pushing_types::Parameters::gamma_b(params, position) };
    const double gamma_t{ pushing_types::Parameters::gamma_t(params, position) };

    const bool in_cone{ gamma_b * vn <= vt and vt <= gamma_t * vn };

    return in_cone;
  }

  virtual Eigen::VectorXd evaluateError(const BodyVelocity& body_vel, const Position& position,
                                        const ContactVelocity& contact_velocity, OptDeriv Hbv = boost::none,
                                        OptDeriv Hpos = boost::none, OptDeriv Hcv = boost::none) const override
  {
    const BodyVelocity predicted_vb{ predict(position, contact_velocity, _params, Hpos, Hcv) };
    const BodyVelocity error{ predicted_vb - body_vel };

    if (Hbv)
    {
      (*Hbv) = -1 * Jacobian::Identity();
    }

    return error;
  }

private:
  const Params _params;
};

// If the contact is outside of the friction cone (the contact point is sliding), this factor
// computes the proportion of that vector that is "inside" the cone (the part that pushes the object)
template <bool SlideUp>
class sliding_pushing_contact_t
  : public gtsam::NoiseModelFactor3<pushing_types::ContactVelocity, pushing_types::Position,
                                    pushing_types::ContactVelocity>
{
  using Base =
      gtsam::NoiseModelFactor3<pushing_types::ContactVelocity, pushing_types::Position, pushing_types::ContactVelocity>;
  using Derived = sliding_pushing_contact_t<SlideUp>;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

public:
  using BodyVelocity = pushing_types::BodyVelocity;
  using Position = pushing_types::Position;
  using ContactVelocity = pushing_types::ContactVelocity;
  using Params = pushing_types::Parameters::Values;

  using Jacobian = Eigen::Matrix<double, 3, 3>;
  using OptDeriv = boost::optional<Eigen::MatrixXd&>;
  using OptJacobian = gtsam::OptionalJacobian<3, 3>;

  sliding_pushing_contact_t(const gtsam::Key key_body_vel, const gtsam::Key key_contact_position,
                            const gtsam::Key key_contact_velocity, const Params params, const NoiseModel& cost_model)
    : Base(cost_model, key_body_vel, key_contact_position, key_contact_velocity), _params(params)
  {
  }

  static ContactVelocity predict(const Position& position, const ContactVelocity& contact_velocity,
                                 const Params& params, OptJacobian Hpos = boost::none, OptJacobian Hvel = boost::none)
  {
    const double& mu{ params[pushing_types::Parameters::mu_idx] };
    const double friction_angle(std::atan(mu));

    const ContactVelocity v_normal{ ContactVelocity(1, 0) };  // Normal vector to contact point
    const ContactVelocity v_up{ Eigen::Rotation2D<double>(-friction_angle) * v_normal };
    const ContactVelocity v_down{ Eigen::Rotation2D<double>(friction_angle) * v_normal };
    const ContactVelocity vb{ SlideUp ? v_up : v_down };

    const double k{ contact_velocity.dot(v_normal) / vb.dot(v_normal) };
    const ContactVelocity v0{ k * vb };

    return v0;
  }

  virtual bool active(const gtsam::Values& values) const override
  {
    const Position position{ this->template key<2>() };
    const ContactVelocity contact_velocity{ this->template key<3>() };
    return active(position, contact_velocity, _params);
  }

  static bool active(const Position& position, const ContactVelocity& contact_velocity, const Params& params)
  {
    const double& vn{ contact_velocity[0] };  // vx, normal
    const double& vt{ contact_velocity[1] };  // vy, tangential

    const double gamma_b{ pushing_types::Parameters::gamma_b(params, position) };
    const double gamma_t{ pushing_types::Parameters::gamma_t(params, position) };

    if constexpr (SlideUp)
    {
      const bool up{ gamma_t * vn < vt };  // up
      return up;
    }
    else
    {
      const bool down{ vt < gamma_b * vn };  // down
      return down;
    }

    return false;
  }

  virtual Eigen::VectorXd evaluateError(const ContactVelocity& sticking_contact_velocity, const Position& position,
                                        const ContactVelocity& contact_velocity, OptDeriv Hbv = boost::none,
                                        OptDeriv Hpos = boost::none, OptDeriv Hcv = boost::none) const override
  {
    const BodyVelocity predicted_vb{ predict(position, contact_velocity, _params, Hpos, Hcv) };
    const BodyVelocity error{ predicted_vb - sticking_contact_velocity };

    if (Hbv)
    {
      (*Hbv) = -1 * Jacobian::Identity();
    }

    return error;
  }

private:
  const Params _params;
};

// Get the component of the contact velocity that produces a sliding due to being outside the contact cone
class sliping_contact_velocity_t
  : public gtsam::NoiseModelFactor3<pushing_types::ContactVelocity, pushing_types::ContactVelocity,
                                    pushing_types::ContactVelocity>
{
  using Base = gtsam::NoiseModelFactor3<pushing_types::ContactVelocity, pushing_types::ContactVelocity,
                                        pushing_types::ContactVelocity>;
  using Derived = sliping_contact_velocity_t;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

public:
  using BodyVelocity = pushing_types::BodyVelocity;
  using Position = pushing_types::Position;
  using ContactVelocity = pushing_types::ContactVelocity;
  using Params = pushing_types::Parameters::Values;

  using Jacobian = Eigen::Matrix<double, 3, 3>;
  using OptDeriv = boost::optional<Eigen::MatrixXd&>;
  using OptJacobian = gtsam::OptionalJacobian<3, 3>;

  sliping_contact_velocity_t(const gtsam::Key key_slipping_vel, const gtsam::Key key_sticking_vel,
                             const gtsam::Key key_contact_velocity, const NoiseModel& cost_model)
    : Base(cost_model, key_slipping_vel, key_sticking_vel, key_contact_velocity)
  {
  }

  static ContactVelocity predict(const ContactVelocity& sticking_velocity, const ContactVelocity& contact_velocity,
                                 OptJacobian Hsv0 = boost::none, OptJacobian Hcv = boost::none)
  {
    const ContactVelocity vslip{ contact_velocity - sticking_velocity };
    return vslip;
  }

  virtual Eigen::VectorXd evaluateError(const ContactVelocity& sliping_velocity,
                                        const ContactVelocity& sticking_velocity,
                                        const ContactVelocity& contact_velocity, OptDeriv Hsl = boost::none,
                                        OptDeriv Hstv = boost::none, OptDeriv Hcv = boost::none) const override
  {
    const ContactVelocity predicted_sv{ predict(sticking_velocity, contact_velocity, Hstv, Hcv) };
    const ContactVelocity error{ predicted_sv - sliping_velocity };

    if (Hsl)
    {
      (*Hsl) = -1 * Jacobian::Identity();
    }

    return error;
  }

private:
};

}  // namespace fg
}  // namespace prx