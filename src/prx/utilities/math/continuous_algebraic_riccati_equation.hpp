#pragma once

#include <Eigen/Core>
#include "prx/utilities/general/constants.hpp"
#include "prx/utilities/general/prx_assert.hpp"

namespace prx
{

/**
** http://et.engr.iupui.edu//~skoskie/ECE684/LNotes/Riccati_algorithms.pdf
**/

class care
{
public:
  template <typename MatrixA, typename MatrixB, typename MatrixQ, typename MatrixR>
  static Eigen::MatrixXd solve(const MatrixA& A, const MatrixB& B, const MatrixQ& Q, const MatrixR& R,
                               std::size_t max_iterations = 100)
  {
    Eigen::LLT<MatrixR> R_c(R);
    if (R_c.info() != Eigen::Success)
      prx_throw("R must be positive definite");

    const Eigen::Index n{ B.rows() };
    const Eigen::Index m{ B.cols() };

    prx_assert(A.rows() == n && A.cols() == n, "Wrong A matrix dimensions");
    prx_assert(Q.rows() == n && Q.cols() == n, "Wrong Q matrix dimensions");
    prx_assert(R_c.matrixL().rows() == m && R_c.matrixL().cols() == m, "Wrong R matrix dimensions");
#if EIGEN_WORLD_VERSION >= 3 && EIGEN_MAJOR_VERSION >= 4
    prx_assert(!std::isnan(A.template maxCoeff<Eigen::PropagateNaN>()), "[CARE] A Matrix contains NaNs!" << A);
    prx_assert(!std::isnan(B.template maxCoeff<Eigen::PropagateNaN>()), "[CARE] B Matrix contains NaNs!" << B);
#endif

    Eigen::MatrixXd H(2 * n, 2 * n);

    H << A, B * R_c.solve(B.transpose()), Q, -A.transpose();

    Eigen::MatrixXd Z{ H };
    Eigen::MatrixXd Z_old{};

    double relative_norm{ 0.0 };
    std::size_t iteration{ 0 };

    const double p{ -1.0 / static_cast<double>(Z.rows()) };

    do
    {
      Z_old = Z;
      const double ck{ std::pow(std::abs(Z.determinant()), p) };
      Z *= ck;
      Z = Z - 0.5 * (Z - Z.inverse());
      relative_norm = (Z - Z_old).norm();
      iteration++;
    } while (iteration < max_iterations && relative_norm > PRX_EPSILON);

// Eigen::PropagateNaN is only in Eigen > 3.4...
#if EIGEN_WORLD_VERSION >= 3 && EIGEN_MAJOR_VERSION >= 4
    prx_assert(!std::isnan(Z.template maxCoeff<Eigen::PropagateNaN>()), "CARE: Matrix contains NaNs!");
#endif

    Eigen::MatrixXd W11{ Z.block(0, 0, n, n) };
    Eigen::MatrixXd W12{ Z.block(0, n, n, n) };
    Eigen::MatrixXd W21{ Z.block(n, 0, n, n) };
    Eigen::MatrixXd W22{ Z.block(n, n, n, n) };

    Eigen::MatrixXd lhs(2 * n, n);
    Eigen::MatrixXd rhs(2 * n, n);
    Eigen::MatrixXd eye = Eigen::MatrixXd::Identity(n, n);
    lhs << W12, W22 + eye;
    rhs << W11 + eye, W21;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(lhs, Eigen::ComputeThinU | Eigen::ComputeThinV);

    auto svd_sol = svd.solve(rhs);
    return svd_sol;
  }
};

}  // namespace prx