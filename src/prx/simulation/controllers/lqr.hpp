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

  lqr_t(const std::size_t& xdim, const std::size_t& udim)
    : _A(MatrixA::Zero(xdim, xdim))
    , _B(MatrixB::Zero(xdim, udim))
    , _Q(MatrixQ::Zero(xdim, xdim))
    , _R(MatrixR::Zero(udim, udim))
    , _K(MatrixK::Zero(udim, xdim))
  {
  }

  template <Eigen::Index InputDim = Xdim, std::enable_if_t<(InputDim != Eigen::Dynamic), bool> = true>
  lqr_t() : lqr_t(Xdim, Udim)
  {
  }

  template <typename MatA, typename MatB, typename MatQ, typename MatR, typename Xref>
  lqr_t(const MatA a, const MatB b, const MatQ q, const MatR r, const Xref x_ref)
    : _A(a), _B(b), _Q(q), _R(r), _K(MatrixK::Zero())
  {
    compute_K();
  }

  template <typename MatA, typename MatB, typename MatQ, typename MatR>
  lqr_t(const MatA a, const MatB b, const MatQ q, const MatR r) : lqr_t(a, b, q, r, VectorX::Zero())
  {
  }

  template <typename MatK>
  lqr_t(const MatK k) : _A(MatrixA::Zero()), _B(MatrixB::Zero()), _Q(MatrixQ::Zero()), _R(MatrixR::Zero()), _K(k)
  {
  }

  virtual ~lqr_t(){};

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

  void compute_K()
  {
    _K = _R.inverse() * (_B.transpose() * S());
  }

  // Computes the control u = -K * X;
  inline VectorU operator()(const VectorX& x) const
  {
    return -_K * x;
  }

  // Computes the control u = -K * (X-X_ref);
  inline VectorU operator()(const VectorX& x, const VectorX& x_ref) const
  {
    return -_K * (x - x_ref);
  }

protected:
  MatrixA _A;
  MatrixB _B;

  MatrixQ _Q;
  MatrixR _R;

  MatrixK _K;
};
}  // namespace simulation
}  // namespace prx