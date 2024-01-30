#pragma once

#include "prx/simulation/plant.hpp"

namespace prx
{
class mushr_t : public plant_t
{
  using Position = Eigen::Vector<double, 3>;
  using Control = Eigen::Vector<double, 3>;
  using Velocity = Eigen::Vector<double, 3>;
  using Transform = Eigen::Transform<double, 3, Eigen::TransformTraits::Isometry>;
  using Rotation = Eigen::Rotation2D<double>;
  using Twist = Eigen::Vector<double, 6>;

public:
  mushr_t(const std::string& path);
  virtual ~mushr_t();

  virtual void propagate(const double simulation_step) override final;

  virtual void update_configuration() override;

protected:
  virtual void compute_derivative() override final;

  inline double desired_velocity() const
  {
    return _u[1];
  }

  inline double steering() const
  {
    return _u[0] * _steering_gain + _steering_offset;
  }

  inline Eigen::Matrix3d hat(const Eigen::Vector3d vec)
  {
    Eigen::Matrix3d mat{};
    mat << 0.0, -vec[2], vec[1],  // no-lint
        vec[2], 0.0, -vec[0],     // no-lint
        -vec[1], vec[0], 0.0;     // no-lint
    return mat;
  }

  inline Eigen::Vector3d vee(const Eigen::Matrix3d mat)
  {
    return Eigen::Vector3d{ mat(2, 1), mat(0, 2), mat(1, 0) };
  }

  inline Eigen::Matrix<double, 6, 6> adjoint(const Transform transform)
  {
    Eigen::Matrix<double, 6, 6> mat{ Eigen::Matrix<double, 6, 6>::Zero() };
    mat.block<3, 3>(0, 0) = transform.linear();
    mat.block<3, 3>(3, 0) = hat(transform.translation()) * transform.linear();
    mat.block<3, 3>(3, 3) = transform.linear();
    return mat;
  }
  inline Eigen::Matrix<double, 4, 4> matrix_twist(const Twist twist)
  {
    Eigen::Matrix<double, 4, 4> mat{ Eigen::Matrix<double, 4, 4>::Zero() };
    mat.block<3, 3>(0, 0) = hat(twist.head(3));
    mat.block<3, 1>(0, 3) = twist.tail(3);
    return mat;
  }

  Position _position;
  double _theta;

  Control _u;

  Velocity _linear_v;
  double _omega;

  Transform _T;
  Transform _Tdot;
  Rotation _rotation;

  Twist _body_twist;
  Twist _spatial_twist;

  Eigen::Matrix<double, 3, 2> _g;
  Eigen::Vector3d _qdot;
  double _length;

  double _current_vel;
  double _vel_delta;
  double _vel_delta_max;
  double _steering_offset;
  double _steering_gain;
  // double _steering_offset, _steering_gain;
};
}  // namespace prx

PRX_REGISTER_SYSTEM(mushr_t, mushr)
