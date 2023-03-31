#pragma once

#include "prx/simulation/plant.hpp"

// Taken from:
//	 @INPROCEEDINGS{7577009,
// 		author={A. {Arab} and K. {Yu} and J. {Yi} and Y. {Liu}},
// 		booktitle={2016 IEEE International Conference on Advanced Intelligent Mechatronics (AIM)},
// 		title={Motion control of autonomous aggressive vehicle maneuvers},
// 		year={2016},
// 		volume={},
// 		number={},
// 		pages={1663-1668},}
namespace prx
{

class racecar_mini_t : public plant_t
{
public:
  using Q = Eigen::Vector3d;
  using QDot = Eigen::Vector3d;
  using QDDot = Eigen::Vector3d;
  using Mass = Eigen::Matrix3d;
  using Coriolis = Eigen::Vector3d;  // simplifying: Coriolis = C(q,qdot)*qdot
  using BMatrix = Eigen::Matrix<double, 3, 4>;
  using Forces = Eigen::Vector4d;
  using Mu = Eigen::Array4d;

  racecar_mini_t(const std::string& path);
  virtual ~racecar_mini_t();

  virtual void propagate(const double simulation_step) override final;

  virtual void update_configuration() override;

  void set_state_space_bounds(const std::vector<double>& lower, const std::vector<double>& upper) override;

  inline Mass& M()
  {
    _M(0, 0) = _mass;
    _M(1, 1) = _mass;
    _M(2, 2) = _iz;
    return _M;
  }

  inline Coriolis& C()
  {
    Eigen::Rotation2D R(_q[2]);
    // Eigen::AngleAxisd angle(_q[2], Eigen::Vector3d::UnitZ());
    // R.linear() = (quaternion_t(cos(_q[2] / 2.), 0, 0, sin(_q[2] / 2.)).toRotationMatrix());

    const Eigen::Vector2d v{ R * _qdot.head(2) };
    PRX_DEBUG_VAR_1(R.toRotationMatrix());
    PRX_DEBUG_VAR_2(_qdot.transpose(), v.transpose());
    // _C[0] = -_mass * _qdot[1] * _qdot[2];
    _C[0] = -_mass * v[1] * _qdot[2];
    _C[1] = _mass * v[0] * _qdot[2];
    _C[2] = 0;
    return _C;
  }

  inline BMatrix& Bx()
  {
    const double c_delta{ std::cos(_delta) };
    const double s_delta{ std::sin(_delta) };
    const double ls{ _L1 * s_delta };
    const double wc{ _W * c_delta };
    _Bx << c_delta, c_delta, 1, 1,  // no-lint
        s_delta, s_delta, 0, 0,     // no-lint
        ls - wc, ls + wc, -_W, _W;  // no-lint
    return _Bx;
  }

  inline BMatrix& By()
  {
    const double c_delta{ std::cos(_delta) };
    const double s_delta{ std::sin(_delta) };
    const double lc{ _L1 * c_delta };
    const double ws{ _W * s_delta };
    _By << s_delta, s_delta, 0, 1,     // no-lint
        c_delta, c_delta, 1, 1,        // no-lint
        lc - ws, lc + ws, -_L2, -_L2;  // no-lint
    return _By;
  }

  inline Forces& Fz()
  {
    // _Fz = _Fx.array() / _mu_x;
    const double L{ _L1 + _L2 };
    _Fz[0] = _mass * _G * _L2 / (2.0 * L) - (_mass * _H / 2.0) * ((_qddot[0] / L) + (_qddot[1] / _W));
    _Fz[1] = _mass * _G * _L2 / (2.0 * L) - (_mass * _H / 2.0) * ((_qddot[0] / L) - (_qddot[1] / _W));
    _Fz[2] = _mass * _G * _L1 / (2.0 * L) - (_mass * _H / 2.0) * ((_qddot[0] / L) - (_qddot[1] / _W));
    _Fz[3] = _mass * _G * _L1 / (2.0 * L) - (_mass * _H / 2.0) * ((_qddot[0] / L) + (_qddot[1] / _W));
    return _Fz;
  }
  inline Forces& Fy()
  {
    _Fy = _Fz.array() * _mu_y;
    // _Fy = Forces::Zero();
    return _Fy;
  }
  inline Forces& Fx()
  {
    // _Fx = Forces::Ones() * _accel / 4;
    // _Fx = (_accel / 4) * (Forces::Ones() - (_mu_x * _mass * gravity * radius) / (4. * stall_torque));
    _Fx = _Fz.array() * _mu_x;
    _Fx += Forces::Ones() * _accel / 4;
    return _Fx;
  }
  // inline double lambda_f()
  // {
  // return (_radius_wheel * _omega - _wheel_velocity);
  // }

protected:
  virtual void compute_derivative() override final;
  const double stall_torque = 0.6;
  const double radius = 0.03;
  const double gravity = 9.81;

  Q _q;
  QDot _qdot;
  QDDot _qddot;

  Mass _M;
  Coriolis _C;
  BMatrix _Bx, _By;

  // Parameters
  double _mass{ 6 };         // mass of the car
  double _iz{ .25 };         // inertia on Z axis

  const double _W{ 0.23 };   // distance between left and right wheels
  const double _L1{ 0.15 };  // distance between vehicle center and front wheel
  const double _L2{ 0.15 };  // distance between vehicle center and back wheel
  const double _H{ 0.25 };

  Mu _mu_x;
  Mu _mu_y;

  // Inputs / Controls
  double _delta;            // Steering
  double _accel;            // Wheel motor input

  Forces _Fx;               // Tire forces on X
  Forces _Fy;               // Tire forces on Y
  Forces _Fz;               // Tire forces on Z

  const double _G{ 9.81 };  // gravity
  // double x, y, theta;
  // double dx, dy, d_theta;
  // double v_linear, v_angular;

  // state
  // double _x;
  // double _y;
  // double _xdot;
  // double _ydot;
  // double _psi;
  // double _psidot;
  // double _wFr;
  // double _wFl;
  // double _wRr;
  // double _wRl;

  // // control
  // double _delta;
  // double _Fflx;
  // double _Ffrx;
  // double _Frlx;
  // double _Frrx;

  // // derivatives
  // // double d_x;
  // // double d_y;
  // double _xdotdot;
  // double _ydotdot;
  // // double d_theta;
  // double _psidotdot;
  // double _wFrdot;
  // double _wFldot;
  // double _wRrdot;
  // double _wRldot;

  // static constexpr double g = 9.81;

  // // mini!?
  // // static constexpr double M = 6.0;
  // // static constexpr double IZ = 0.25;
  // static constexpr double L1 = 0.2;
  // static constexpr double L2 = 0.2;
  // static constexpr double W = 0.15;
  // static constexpr double R = 0.025;
  // // static constexpr double IF = 1.8;
  // // static constexpr double IR = 1.8;
  // static constexpr double B = 2;
  // static constexpr double C = 0.6;
  // static constexpr double D = 0.05;

  // static constexpr double L = L1 + L2;
  // static constexpr double d_SV = 1.;

  std::vector<double> lower_bound = { -26, -24, -3.14159, -5, -5, -5 };
  std::vector<double> upper_bound = { 26, 24, 3.14159, 5, 5, 5 };
};
}  // namespace prx
PRX_REGISTER_SYSTEM(racecar_mini_t, racecar_mini)
