#include "prx/simulation/controllers/lqr.hpp"

namespace prx
{

lqr_t::~lqr_t()
{
}

void lqr_t::compute_K()
{
  const std::size_t ss_dim{ plant->get_state_space()->get_dimension() };
  const std::size_t cs_dim{ plant->get_control_space()->get_dimension() };
  Eigen::MatrixXd A{ Eigen::MatrixXd::Zero(ss_dim, ss_dim) };
  Eigen::MatrixXd B{ Eigen::MatrixXd::Zero(cs_dim, cs_dim) };
  plant->linearize(A, B);
  // PRX_DEBUG_VAR_1(B);
  // PRX_DEBUG_VAR_1(A);
  Eigen::MatrixXd S = care::solve(A, B, Q, R);
  K = R.inverse() * (B.transpose() * S);
}

void lqr_t::compute_controls()
{
  prx_assert(K.rows() > 0 && K.cols() > 0, "Gain matrix (K) has not been computed! Call lqr_t -> compute_K() needed.");
  plant->get_state_space()->copy_to(X);
  U = -K * (X - X_goal);
  // std::cout << "X: " << X.transpose() << "\tU: " << U << std::endl;
  plant->get_control_space()->copy_from(U);
  plant->get_control_space()->enforce_bounds();
}

}  // namespace prx