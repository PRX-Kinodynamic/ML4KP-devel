#define BOOST_AUTO_TEST_MAIN quat_euler_tests
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/general/transforms.hpp"

using Quaternion = Eigen::Quaterniond;
using AngleAxis = Eigen::AngleAxisd;
using Vector3d = Eigen::Vector3d;

Quaternion euler_to_quat(const double x, const double y, const double z)
{
  return AngleAxis(x, Vector3d::UnitX()) *  // no-lint
         AngleAxis(y, Vector3d::UnitY()) *  // no-lint
         AngleAxis(z, Vector3d::UnitZ());
}

Vector3d quat_to_euler(const Quaternion& quat)
{
  return quat.toRotationMatrix().eulerAngles(0, 1, 2);
}

Vector3d QuaternionToAxisAngle(const Quaternion& q)
{
  // roll (x-axis rotation)
  const double sinr_cosp{ 2 * (q.w() * q.x() + q.y() * q.z()) };
  const double cosr_cosp{ 1 - 2 * (q.x() * q.x() + q.y() * q.y()) };
  const double x{ std::atan2(sinr_cosp, cosr_cosp) };

  // pitch (y()-ax()is rotation)
  const double sinp{ std::sqrt(1 + 2 * (q.w() * q.y() - q.x() * q.z())) };
  const double cosp{ std::sqrt(1 - 2 * (q.w() * q.y() - q.x() * q.z())) };
  const double y{ 2 * std::atan2(sinp, cosp) - M_PI / 2.0 };

  // yaw() (z-ax()is rotation)
  const double siny_cosp{ 2 * (q.w() * q.z() + q.x() * q.y()) };
  const double cosy_cosp{ 1 - 2 * (q.y() * q.y() + q.z() * q.z()) };
  const double z{ std::atan2(siny_cosp, cosy_cosp) };

  return { x, y, z };
}

BOOST_AUTO_TEST_CASE(quat_euler_z_test)
{
  const double x{ 0 };
  // const double y{ 0 };
  const double angle_step{ 0.1 };
  const double angle_step_high{ 0.01 };
  for (double y = -0.03; y < 0.03; y += angle_step_high)
  {
    for (double z = -M_PI; z < M_PI; z += angle_step)
    {
      const Quaternion quat{ euler_to_quat(x, y, z) };
      const Vector3d angles{ quat_to_euler(quat) };
      std::cout << x << " " << y << "  " << z << " ";
      std::cout << quat.coeffs().transpose() << " " << angles.transpose() << " ";
      std::cout << QuaternionToAxisAngle(quat).transpose() << " ";
      std::cout << "\n";
    }
  }
}