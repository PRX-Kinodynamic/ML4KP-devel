#pragma once

#include "prx/utilities/math/continuous_algebraic_riccati_equation.hpp"

namespace prx
{
// I would prefer to hace a "control" namespace / directory
namespace simulation
{

template <Eigen::Index Xdim, Eigen::Index Udim>
class lqr_t
{
public:
  using MatrixA = Eigen::Matrix<double, Xdim, Xdim>;
  using MatrixB = Eigen::Matrix<double, Xdim, Udim>;

  using MatrixQ = Eigen::Matrix<double, Xdim, Xdim>;
  using MatrixR = Eigen::Matrix<double, Udim, Udim>;

  using MatrixK = Eigen::Matrix<double, Udim, Xdim>;

  using VectorX = Eigen::Vector<double, Xdim>;
  using VectorU = Eigen::Vector<double, Udim>;

  using Diff = std::function<VectorX(const VectorX&, const VectorX&)>;

  inline static Diff DefaultDiff = [](const VectorX& a, const VectorX& b) { return a - b; };

  template <typename MatA, typename MatB, typename MatQ, typename MatR, typename MatK>
  lqr_t(const MatA a, const MatB b, const MatQ q, const MatR r, const MatK k, const Diff diff = DefaultDiff)
    : _A(a), _B(b), _Q(q), _R(r), _K(k), _diff(diff)
  {
  }

  template <Eigen::Index InputDim = Xdim, std::enable_if_t<(InputDim == Eigen::Dynamic), bool> = true>
  lqr_t(const std::size_t& xdim, const std::size_t& udim, const Diff diff = DefaultDiff)
    : lqr_t(MatrixA::Identity(xdim, xdim), MatrixB::Identity(xdim, udim), MatrixQ::Identity(xdim, xdim),
            MatrixR::Identity(udim, udim), MatrixK::Zero(udim, xdim), diff)
  {
  }

  template <Eigen::Index InputDim = Xdim, std::enable_if_t<(InputDim != Eigen::Dynamic), bool> = true>
  lqr_t()
    : lqr_t(MatrixA::Identity(Xdim, Xdim), MatrixB::Identity(Xdim, Udim), MatrixQ::Identity(Xdim, Xdim),
            MatrixR::Identity(Udim, Udim), MatrixK::Zero(Udim, Xdim), DefaultDiff)
  {
  }

  template <typename MatA, typename MatB, typename MatQ, typename MatR>
  lqr_t(const MatA a, const MatB b, const MatQ q, const MatR r) : lqr_t(a, b, q, r, MatrixK::Zero(), DefaultDiff)
  {
    compute_K();
  }

  template <typename MatK>
  lqr_t(const MatK k)
    :  // _A(MatrixA::Identity(Xdim, Xdim))
       // , _B(MatrixB::Identity(Xdim, Udim))
       // , _Q(MatrixQ::Zero(Xdim, Xdim))
       // , _R(MatrixR::Zero(Udim, Udim))
    _K(k)
    , _diff(DefaultDiff)
  {
  }

  virtual ~lqr_t() {};

  MatrixQ Q() const
  {
    return _Q;
  }

  MatrixQ& Q()
  {
    return _Q;
  }

  MatrixR R() const
  {
    return _R;
  }

  MatrixR& R()
  {
    return _R;
  }

  MatrixK K() const
  {
    return _K;
  }

  MatrixK& K()
  {
    return _K;
  }

  MatrixA A() const
  {
    return _A;
  }

  MatrixA& A()
  {
    return _A;
  }

  MatrixB B() const
  {
    return _B;
  }

  MatrixB& B()
  {
    return _B;
  }

  Eigen::MatrixXd S()
  {
    return care::solve(_A, _B, _Q, _R);
  }

  // Check {Q,R} >= 0 && R invertible
  // For big matrices, this check might be *slow*
  void compute_K()
  {
    const Eigen::LDLT<MatrixQ> q_ldlt{ _Q };
    prx_assert(q_ldlt.isPositive(), "Matrix Q must be positive semi-definitve");
    const Eigen::LDLT<MatrixR> r_ldlt{ _R };
    prx_assert(r_ldlt.isPositive(), "Matrix R must be positive");
    const Eigen::FullPivLU<MatrixR> r_lu{ _R };
    prx_assert(r_lu.isInvertible(), "Matrix R must be invertible");

    _K = r_lu.inverse() * (_B.transpose() * S());
  }

  // Computes the control u = -K * X;
  inline VectorU operator()(const VectorX& x) const
  {
    return this->operator()(x, VectorX::Zero(x.size()));
  }

  // Computes the control u = -K * (X-X_ref);
  inline VectorU operator()(const VectorX& x, const VectorX& x_ref) const
  {
    const VectorX df{ _diff(x, x_ref) };
    return -_K * df;
    // return -_K * (x - x_ref);
  }

protected:
  MatrixA _A;
  MatrixB _B;

  MatrixQ _Q;
  MatrixR _R;

  MatrixK _K;

  const Diff _diff;
};
}  // namespace simulation
}  // namespace prx