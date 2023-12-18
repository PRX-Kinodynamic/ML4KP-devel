#pragma once

/**
 * @file transforms.hpp
 * @author Zakary Littlefield
 * @brief <b> [INCOMPLETE] Some handy aliases for commonly used Eigen declarations. </b>
 * */
#include "prx/utilities/general/prx_assert.hpp"
#include <Eigen/Dense>
#include <Eigen/Core>

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

template <typename EigenMatrix, typename Container>
void init_from_container(EigenMatrix& matrix, const Container& container)
{
  const Eigen::Index rows{ matrix.rows() };
  const Eigen::Index cols{ matrix.cols() };
  prx_assert(container.size() == rows * cols, "Dimensions doesn't match. Container has "
                                                  << std::to_string(container.size()).c_str() << " but Matrix has ( "
                                                  << rows << ", " << cols << ").");
  for (int i = 0; i < rows; ++i)
  {
    for (int j = 0; j < cols; ++j)
    {
      matrix(i, j) = container[i * cols + j];
    }
  }
}

// void quaternion_to_euler_012(const Eigen::Quaterniond& q, Eigen::Vector3d& out_vec)
// {
//   // roll (x-axis rotation)
//   const double sinr_cosp{ 2 * (q.w() * q.x() + q.y() * q.z()) };
//   const double cosr_cosp{ 1 - 2 * (q.x() * q.x() + q.y() * q.y()) };
//   out_vec[0] = std::atan2(sinr_cosp, cosr_cosp);

//   // pitch (y()-ax()is rotation)
//   const double sinp{ std::sqrt(1 + 2 * (q.w() * q.y() - q.x() * q.z())) };
//   const double cosp{ std::sqrt(1 - 2 * (q.w() * q.y() - q.x() * q.z())) };
//   out_vec[1] = 2 * std::atan2(sinp, cosp) - M_PI / 2.0;

//   // yaw() (z-ax()is rotation)
//   const double siny_cosp{ 2 * (q.w() * q.z() + q.x() * q.y()) };
//   const double cosy_cosp{ 1 - 2 * (q.y() * q.y() + q.z() * q.z()) };
//   out_vec[2] = std::atan2(siny_cosp, cosy_cosp);
// }
}  // namespace prx