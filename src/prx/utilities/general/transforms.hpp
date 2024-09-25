#pragma once

/**
 * @file transforms.hpp
 * @author Zakary Littlefield
 * @brief <b> [INCOMPLETE] Some handy aliases for commonly used Eigen declarations. </b>
 * */

#include <Eigen/Dense>
#include <Eigen/Core>
#include "prx/utilities/general/prx_assert.hpp"
#include <yaml-cpp/yaml.h>
namespace prx
{
/** @brief A column vector of dimension <i>N</i>. */
template <int N>
using n_vector_t = Eigen::Matrix<double, N, 1>;
/** @brief A matrix of dimension <i>N</i> x <i>N</i>. */
template <int N>
using n_matrix_t = Eigen::Matrix<double, N, N>;
/** @brief A column vector of dimension 3. */
using vector_t = n_vector_t<3>;
/** @brief A 3x3 matrix. */
using matrix_t = n_matrix_t<3>;
/** @brief A quaternion. */
using quaternion_t = Eigen::Quaternion<double>;
using axis_angle_t = Eigen::AngleAxis<double>;
using transform_t = Eigen::Transform<double, 3, Eigen::AffineCompact>;

using ref_matrixXd_t = Eigen::Ref<const Eigen::MatrixXd>;

inline transform_t create_transform(const matrix_t rotation, const vector_t translation)
{
  transform_t tf{};
  tf.linear() = rotation;
  tf.translation() = translation;
  return tf;
}

// template<typename Derived>
// static
inline bool are_matrices_approx_equal(const ref_matrixXd_t m1, const ref_matrixXd_t m2, const double tolerance = 1e-7)
{
  // prx_assert(m1.rows() == m2.rows(), "Matrices must have equal dimensions!");
  // prx_assert(m1.cols() == m2.cols(), "Matrices must have equal dimensions!");

  for (int i = 0; i < m1.rows(); ++i)
  {
    for (int j = 0; j < m1.cols(); ++j)
    {
      if (!(std::fabs(m1(i, j) - m2(i, j)) <= tolerance))
        return false;
    }
  }
  return true;
}

inline Eigen::Vector3d quaternion_to_euler(const quaternion_t& q)
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

template <typename Rotation, typename Angles>
inline Rotation euler_to_rotation(const Angles& angles, const std::string order)
{
  Eigen::Matrix3d R{ Eigen::Matrix3d::Identity() };
  prx_assert(angles.size() == order.size(), "Mismatch on sizes");
  const std::size_t size{ order.size() };

  for (int i = 0; i < size; ++i)
  {
    const double angle{ angles[i] };
    const char axis{ order[i] };
    switch (axis)
    {
      case 'X':
        R = R * Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX());
        break;
      case 'Y':
        R = R * Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY());
        break;
      case 'Z':
        R = R * Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ());
        break;
    }
  }

  const Rotation result{ R };
  return result;
}

}  // namespace prx

// Conversions to/from YAML
namespace YAML
{
template <typename Type, Eigen::Index N>
struct convert<Eigen::Vector<Type, N>>
{
  using Vector = Eigen::Vector<Type, N>;
  static Node encode(const Vector& rhs)
  {
    Node node;
    for (int i = 0; i < rhs.size(); ++i)
    {
      node.push_back(rhs[i]);
    }
    return node;
  }

  template <Eigen::Index InputDim = N, std::enable_if_t<(InputDim != Eigen::Dynamic), bool> = true>
  static bool decode(const Node& node, Vector& lhs)
  {
    return decode_imp(node, lhs);
  }

  template <Eigen::Index InputDim = N, std::enable_if_t<(InputDim == Eigen::Dynamic), bool> = true>
  static bool decode(const Node& node, Vector& lhs)
  {
    lhs = Vector::Zero(node.size());
    return decode_imp(node, lhs);
  }

  static bool decode_imp(const Node& node, Vector& lhs)
  {
    const std::size_t vec_size{ static_cast<std::size_t>(lhs.size()) };
    if (!node.IsSequence() || node.size() != vec_size)
    {
      return false;
    }
    for (int i = 0; i < vec_size; ++i)
    {
      lhs[i] = node[i].as<double>();
    }
    return true;
  }
};
}  // namespace YAML