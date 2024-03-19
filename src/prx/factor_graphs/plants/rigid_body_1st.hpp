#pragma once

#include "prx/simulation/plant.hpp"

namespace prx
{
namespace fg
{

// x_i + \dot{x}_i -> x_j
class kinematic_rigid_body_factors_t : public gtsam::NoiseModelFactor3<se3_t, screw_axis_t, se3_t>
{
  using Base = gtsam::NoiseModelFactor3<se3_t, screw_axis_t, se3_t>;
  using NoiseModelPtr = gtsam::noiseModel::Base::shared_ptr;

public:
  rigid_body_position_velocity_t(const gtsam::Key x_i, const gtsam::Key xdot, const gtsam::Key x_j, const double dt,
                                 const NoiseModelPtr& cost_model)
    : Base(x_i, xdot, x_j, control, parameters, cost_model), _dt(dt)
  {
  }

  static se3_t predict(const se3_t& xi, const screw_axis_t& screw, const double& dt,  // no-lint
                       gtsam::OptionalJacobian<6, 6> Hxi = boost::none,
                       gtsam::OptionalJacobian<6, 6> Hscrew = boost::none)
  {
    lie_integrator_t::predict(xi, screw, dt, _dt, Hxi, Hscrew, Hxj);
  }

  virtual State compute_error(const se3_t& xi, const screw_axis_t& screw, const se3_t& xj,  // no-lint
                              gtsam::OptionalJacobian<6, 6> Hxi = boost::none,
                              gtsam::OptionalJacobian<6, 6> Hscrew = boost::none,
                              gtsam::OptionalJacobian<6, 6> Hxj = boost::none) const override
  {
    if (Hxj)
    {
      *Hxj = -Eigen::Matrix<double, 6, 6>::Identity();
    }
    return predict(xi, screw, _dt, Hxi, Hscrew, Hxj) - xj;
  }

  void header_to_stream(std::ostream& os)
  {
    os << "dt" << sp;
    os << "xi" << sp;
    os << "screw" << sp;
    os << "xj" << sp;
    os << "\n";
  }

  void values_to_stream(gtsam::Values& values, std::ostream& os)
  {
    const se3_t xi{ values.at<State>(key<1>()) };
    const screw_axis_t screw{ values.at<Control>(key<2>()) };
    const se3_t xj{ values.at<State>(key<3>()) };

    const char sp{ prx::constants::separating_value };
    // const double
    os << _dt << sp;                //  1,
    os << xi.transpose() << sp;     //  2,  3,  4,  5,  6,  7, 8
    os << screw.transpose() << sp;  //  9, 10, 11, 12, 13, 14,
    os << xj.transpose() << sp;     // 15, 16, 17, 18, 19, 20, 21
    os << "\n";
  }

private:
  const double _dt;
};

class kinematic_rigid_body_t : public plant_t
{
public:
  rigid_body_1st_t(const std::string& path) : plant_t(path), _pose()
  {
    state_memory = {, &_pose[4], &_pose[5], &_pose[6], &_pose[0], &_pose[1], &_pose[2], &_pose[3] };
    state_space = new space_t("EEEQQQQ", state_memory, "rigid_body_1st_state");

    control_memory = { &_screw[0], &_screw[1], &_screw[2], &_screw[3], &_screw[4], &_screw[5] };
    input_control_space = new space_t("EEEEEE", control_memory, "Torque");

    geometries["body"] = std::make_shared<geometry_t>(geometry_type_t::BOX);
    geometries["body"]->initialize_geometry({ 1, 1, 1 });
    geometries["body"]->generate_collision_geometry();
    geometries["body"]->set_visualization_color("0x00ff00");
    configurations["body"] = std::make_shared<transform_t>();
    configurations["body"]->setIdentity();
  }

  virtual ~rigid_body_1st_t()
  {
  }

  virtual void propagate(const double simulation_step) override final
  {
    kinematic_rigid_body_factors_t::predict(_pose, _screw, simulation_step);
  }

  virtual void update_configuration() override
  {
    body->linear() = _pose.matrix();
    body->translation() = _pose.position();
  }

protected:
  se3_t _pose;
  screw_axis_t _screw;
};
}  // namespace fg
}  // namespace prx
PRX_REGISTER_SYSTEM(rigid_body_1st_t, rigid_body_1st)
