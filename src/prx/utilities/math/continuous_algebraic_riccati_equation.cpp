#include "prx/utilities/math/continuous_algebraic_riccati_equation.hpp"

namespace prx
{

    Eigen::MatrixXd care::solve(const ref_matrixXd_t& A, const ref_matrixXd_t& B, 
        const ref_matrixXd_t& Q, const ref_matrixXd_t& R, unsigned int max_iterations) 
    {

        Eigen::LLT<Eigen::MatrixXd> R_c(R);
        if (R_c.info() != Eigen::Success)
            prx_throw("R must be positive definite");

        auto n = B.rows();
        auto m = B.cols();

        // std::cout << "CARE B:" << B << std::endl;
        // std::cout << "CARE Q:" << Q << std::endl;
        prx_assert(A.rows() == n && A.cols() == n, "Wrong matrix dimensions");
        prx_assert(Q.rows() == n && Q.cols() == n, "Wrong matrix dimensions");
        prx_assert(R_c.matrixL().rows() == m && R_c.matrixL().cols() == m, "Wrong matrix dimensions");
        
        Eigen::MatrixXd H(2 * n, 2 * n);

        H << A, B * R_c.solve(B.transpose()), Q, -A.transpose();

        Eigen::MatrixXd Z = H;
        Eigen::MatrixXd Z_old;

        double relative_norm;
        unsigned int iteration = 0;

        const double p = static_cast<double>(Z.rows());

        do 
        {
            Z_old = Z;
            double ck = std::pow(std::abs(Z.determinant()), -1.0 / p);
            Z *= ck;
            Z = Z - 0.5 * (Z - Z.inverse());
            relative_norm = (Z - Z_old).norm();
            iteration++;
        } while (iteration < max_iterations && relative_norm > PRX_EPSILON);

        prx_assert( !std::isnan(Z.template maxCoeff<Eigen::PropagateNaN>()), "CARE: Matrix contains NaNs!" );

        Eigen::MatrixXd W11 = Z.block(0, 0, n, n);
        Eigen::MatrixXd W12 = Z.block(0, n, n, n);
        Eigen::MatrixXd W21 = Z.block(n, 0, n, n);
        Eigen::MatrixXd W22 = Z.block(n, n, n, n);

        Eigen::MatrixXd lhs(2 * n, n);
        Eigen::MatrixXd rhs(2 * n, n);
        Eigen::MatrixXd eye = Eigen::MatrixXd::Identity(n, n);
        lhs << W12, W22 + eye;
        rhs << W11 + eye, W21;

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(lhs, Eigen::ComputeThinU | Eigen::ComputeThinV);

        auto svd_sol = svd.solve(rhs);
        return svd_sol;
    }
}

