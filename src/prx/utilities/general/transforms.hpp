#pragma once

/**
 * @file transforms.hpp
 * @author Zakary Littlefield
 * @brief <b> [INCOMPLETE] Some handy aliases for commonly used Eigen declarations. </b>
 * */

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

	template<typename Derived>
	static inline
	bool are_approx_equal(const Eigen::MatrixBase<Derived>& m1, const  Eigen::MatrixBase<Derived> m2, const double tolerance = 1e-7)
	{
		prx_assert(m1.rows() == m2.rows(), "Matrices must have equal dimensions!");
		prx_assert(m1.cols() == m2.cols(), "Matrices must have equal dimensions!");

		for (int i = 0; i < m1.rows(); ++i)
		{
			for (int j = 0; j < m1.cols(); ++j)
			{
				if ( ! ( std::fabs(m1 - m2) <= tolerance ) )
					return false;
			}
		}
		return true;
	}
}