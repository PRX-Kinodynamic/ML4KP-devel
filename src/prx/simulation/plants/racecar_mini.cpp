
#include "prx/simulation/plants/racecar_mini.hpp"

namespace prx
{

racecar_mini_t::racecar_mini_t(const std::string& path)
  : plant_t(path)
  , _q(Q::Zero())
  , _qdot(QDot::Zero())
  , _qddot(QDDot::Zero())
  , _Fx(Forces::Zero())
  , _Fy(Forces::Zero())
  , _M(Mass::Zero())
  , _C(Coriolis::Zero())
  , _Bx(BMatrix::Zero())
  , _By(BMatrix::Zero())
  , _mu_x(Mu::Ones())
  , _mu_y(Mu::Zero())
{
  const double INF{ std::numeric_limits<double>::infinity() };

  state_memory = { &_q[0], &_q[1], &_q[2], &_qdot[0], &_qdot[1], &_qdot[2] };
  state_space = new space_t("EEREEE", state_memory, "XYPsiXdYdPsid");
  state_space->set_bounds(lower_bound, upper_bound);

  derivative_memory = { &_qdot[0], &_qdot[1], &_qdot[2], &_qddot[0], &_qddot[1], &_qddot[2] };
  derivative_space = new space_t("EEEEEE", derivative_memory, "XdYdPsidXddYddPsidd");

  control_memory = { &_delta, &_accel };
  input_control_space = new space_t("RE", control_memory, "steering_motor_accel");
  input_control_space->set_bounds({ -1., -1 }, { 1., 1 });
  // input_control_space->set_bounds({ -1., -15, -15, -15, -15 }, { 1., 15, 15, 15, 15 });

  parameter_memory = { &_mass, &_iz };
  parameter_space = new space_t("EE", parameter_memory, "parameters");
  parameter_space->set_bounds({ 0, 0 }, { INF, INF });

  geometries["body"] = std::make_shared<geometry_t>(geometry_type_t::BOX);
  geometries["body"]->initialize_geometry({ _L1 + _L2, _W, _H });

  geometries["body"]->generate_collision_geometry();
  geometries["body"]->set_visualization_color("0x00ff00");
  configurations["body"] = std::make_shared<transform_t>();
  configurations["body"]->setIdentity();

  set_integrator(integrator_t::kRK4);
}

void racecar_mini_t::set_state_space_bounds(const std::vector<double>& lower, const std::vector<double>& upper)
{
  for (int i = 0; i < std::min(lower.size(), lower_bound.size()); ++i)
  {
    lower_bound[i] = lower[i];
  }
  for (int i = 0; i < std::min(upper.size(), upper_bound.size()); ++i)
  {
    upper_bound[i] = upper[i];
  }

  state_space->set_bounds(lower_bound, upper_bound);
}

racecar_mini_t::~racecar_mini_t()
{
}

void racecar_mini_t::propagate(const double simulation_step)
{
  integrator->integrate(simulation_step);
  PRX_DEBUG_VAR_3(_q.transpose(), _qdot.transpose(), _qddot.transpose());
}

void racecar_mini_t::update_configuration()
{
  auto body = configurations["body"];
  // Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
  // Eigen::AngleAxisd angle(_q[2], Eigen::Vector3d::UnitZ());
  // Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());

  // Eigen::Quaternion<double> q = yawAngle;
  const double theta{ -_q[2] };
  Eigen::AngleAxisd angle(theta, Eigen::Vector3d::UnitZ());
  body->setIdentity();
  body->translation() = (vector_t(_q[0], _q[1], 0));
  // body->linear() = (quaternion_t(cos(theta / 2.), 0, 0, sin(theta / 2.)).toRotationMatrix());
  body->linear() = angle.toRotationMatrix();
}

void racecar_mini_t::compute_derivative()
{
  Fx();
  Fz();
  _qddot = M().inverse() * (Bx() * _Fx + By() * Fy() - C());

  Eigen::Rotation2D R(_q[2]);

  const Eigen::Vector2d v{ R * _qddot.head(2) };
}
}  // namespace prx
