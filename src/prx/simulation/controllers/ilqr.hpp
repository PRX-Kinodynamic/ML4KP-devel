#pragma once

#include <functional>
#include <iostream>
#include "prx/utilities/math/first_order_derivative.hpp"

namespace prx
{
extern double simulation_step;

// Iterative LQR as in:
// Li, Weiwei, and Emanuel Todorov. "Iterative linear quadratic regulator design for nonlinear biological movement
// systems." In First International Conference on Informatics in Control, Automation and Robotics, vol. 2, pp. 222-229.
// SciTePress, 2004.
// Using mostly the same notation (except for indices)
namespace simulation
{
using namespace std::placeholders;

template <Eigen::Index Xdim, Eigen::Index Udim, uint8_t Evaluations, int8_t MinDifference>
class iLQR_state_t
{
public:
  using VectorX = Eigen::Vector<double, Xdim>;
  using VectorU = Eigen::Vector<double, Udim>;

  using VectorV = Eigen::Vector<double, Xdim>;

  using MatrixA = Eigen::Matrix<double, Xdim, Xdim>;
  using MatrixB = Eigen::Matrix<double, Xdim, Udim>;
  using MatrixBt = Eigen::Matrix<double, Udim, Xdim>;

  using MatrixK = Eigen::Matrix<double, Udim, Xdim>;
  using MatrixKt = Eigen::Matrix<double, Xdim, Udim>;  // Transpose of K
  using MatrixKv = Eigen::Matrix<double, Udim, Xdim>;
  using MatrixKu = Eigen::Matrix<double, Udim, Udim>;

  using MatrixQ = Eigen::Matrix<double, Xdim, Xdim>;
  using MatrixR = Eigen::Matrix<double, Udim, Udim>;

  using MatrixS = Eigen::Matrix<double, Xdim, Xdim>;

  using DynamicFunction = std::function<VectorX(const VectorX&, const VectorU&)>;

  using DynamicFunctionPartialX = std::function<VectorX(const VectorX&)>;
  using DynamicFunctionPartialU = std::function<VectorX(const VectorU&)>;

  using DerivA = prx::math::first_order_derivative_t<DynamicFunctionPartialX, VectorX, Evaluations, MinDifference>;
  using DerivB = prx::math::first_order_derivative_t<DynamicFunctionPartialU, VectorU, Evaluations, MinDifference>;

  iLQR_state_t() = delete;

  template <typename DynamicFunctionIn>
  iLQR_state_t(const std::size_t idx, const VectorX& x, const VectorU& u, const DynamicFunctionIn& dynamic_function,
               const MatrixS* S_next, const MatrixQ* Qp, const MatrixR* Rp, const VectorV* v_next)
    : _idx(idx)
    , _x(x)
    , _u(u)
    , _S_next(S_next)
    , _Qp(Qp)
    , _Rp(Rp)
    , _v_next(v_next)
    , _dynamic_function(dynamic_function)
    , _A_deriv(prx::simulation_step, Xdim, Xdim)
    , _B_deriv(prx::simulation_step, Udim, Xdim)
    , _update(true)
  {
    evaluate();
    PRX_DEBUG_VAR_3(_idx, _x.transpose(), _u.transpose());
  }

  VectorU& U()
  {
    return _u;
  }

  VectorX& X()
  {
    return _x;
  }

  MatrixA& A()
  {
    _A_deriv._model = std::bind(_dynamic_function, _1, _u);
    _A = _A_deriv(_x);
    return _A;
  }
  MatrixB& B()
  {
    _B_deriv._model = std::bind(_dynamic_function, _x, _1);
    _B = _B_deriv(_u);
    return _B;
  }

  void evaluate()
  {
    _update = true;
    // PRX_DEBUG_VAR_2(_idx, _A);
    // PRX_DEBUG_VAR_2(_idx, _B);
    // PRX_DEBUG_VAR_2(_idx, (*_S_next));
    S();
    v();
  }

  // VectorX x_next()
  // {
  //   const VectorX x_n{ A() * _x + B() * _u };
  //   PRX_DEBUG_VAR_2(_idx, A());
  //   PRX_DEBUG_VAR_2(_idx, B());
  //   PRX_DEBUG_VAR_3(_idx, _x.transpose(), _u);
  //   PRX_DEBUG_VAR_2(_idx, x_n.transpose());
  //   return x_n;
  // }

  VectorX dx_next(const VectorX& dx)
  {
    const VectorX dx_n{ A() * dx + B() * du(dx) };
    PRX_DEBUG_VAR_2(_idx, dx_n.transpose());
    return dx_n;
  }

  const VectorU du(const VectorX& dx)
  {
    const VectorV v1{ *_v_next };
    _du = -K() * dx - Kv() * v1 - Ku() * _u;
    // PRX_DEBUG_VAR_2(_idx, _du);
    return _du;
  }

  const MatrixK& K()
  {
    const MatrixBt Bt{ B().transpose() };
    const MatrixS S1{ *_S_next };
    const MatrixR R{ *_Rp };
    _K = (Bt * S1 * B() + R).inverse() * Bt * S1 * A();
    // PRX_DEBUG_VAR_2(_idx, _K);
    return _K;
  }

  const MatrixKu& Ku()
  {
    const MatrixBt Bt{ B().transpose() };
    const MatrixS S1{ *_S_next };
    const MatrixR R{ *_Rp };
    _Ku = (Bt * S1 * B() + R).inverse() * R;
    // PRX_DEBUG_VAR_2(_idx, _Ku);
    return _Ku;
  }

  const MatrixKv& Kv()
  {
    const MatrixBt Bt{ B().transpose() };
    const MatrixS S1{ *_S_next };
    const MatrixR R{ *_Rp };
    _Kv = (Bt * S1 * B() + R).inverse() * Bt;
    // PRX_DEBUG_VAR_2(_idx, _Kv);
    return _Kv;
  }

  MatrixS& S()
  {
    const MatrixA At{ A().transpose() };
    const MatrixBt Bt{ B().transpose() };
    const MatrixS S1{ *_S_next };
    const MatrixQ Q{ *_Qp };
    _S = At * S1 * (A() - B() * K()) + Q;
    // PRX_DEBUG_VAR_2(_idx, _S);
    return _S;
  }

  VectorV& v()
  {
    const MatrixA At{ A().transpose() };
    const MatrixBt Bt{ B().transpose() };
    const MatrixKt Kt{ K().transpose() };
    const MatrixQ Q{ *_Qp };
    const MatrixR R{ *_Rp };
    const VectorV v1{ *_v_next };
    _v = (A() - B() * K()).transpose() * v1 - Kt * R * _u + Q * _x;
    PRX_DEBUG_VAR_3(_idx, v1.transpose(), _v.transpose());
    return _v;
  }

private:
  const std::size_t _idx;  // id of this state, \in [0,N]
  MatrixA _A;              // State matrix at idx
  MatrixB _B;              // Control matrix at idx

  MatrixK _K;
  MatrixKv _Kv;
  MatrixKu _Ku;

  MatrixS _S;
  VectorV _v;

  VectorU _u;
  VectorU _du;

  VectorX _x;

  // The following are pointers to avoid expensive copies / storing
  const MatrixS* _S_next;  // S_{i+1}
  const VectorV* _v_next;

  const MatrixQ* _Qp;  // Pointer to Q
  const MatrixR* _Rp;  // Pointer to R

  DynamicFunction _dynamic_function;

  DerivA _A_deriv;
  DerivB _B_deriv;

  bool _update;
};

template <Eigen::Index Xdim, Eigen::Index Udim, uint8_t Evaluations = 5, int8_t MinDifference = -1>
class iLQR_t
{
public:
  using iLQRState = iLQR_state_t<Xdim, Udim, Evaluations, MinDifference>;

  using VectorX = typename iLQRState::VectorX;
  using VectorU = typename iLQRState::VectorU;
  using DynamicFunction = typename iLQRState::DynamicFunction;

  using MatrixQ = typename iLQRState::MatrixQ;
  using MatrixR = typename iLQRState::MatrixR;

  using MatrixS = typename iLQRState::MatrixS;
  using VectorV = typename iLQRState::VectorV;

  template <typename DynamicFunctionIn>
  iLQR_t(const DynamicFunctionIn& dynamic_function, const MatrixQ Q, const MatrixR R, const VectorX xGoal,
         const MatrixQ Qf)
    : _dynamic_function(dynamic_function), _Q(Q), _R(R), _xGoal(xGoal), _Qf(Qf)
  {
    ofs_map.open(prx::out_path + "/dbg_ilqr.txt", std::ofstream::trunc);
  }

  // Expecting a pair trajectory/controls such that:
  // x0    x1    x2    (...)    xN
  //   \  /  \  /  \  /     \  /
  //    u0    u1    u2       u{N-1}
  // This is | trajectory | == | controls | + 1
  // Both trajectory and control objects have to implement (as Containers):
  // 		* reverse iterators
  //	  * size()
  template <typename Controls, typename StateX>
  void update(const Controls controls, const StateX x0, const double epsilon = 0.1)
  {
    init_sequence(controls);
    double dui_cost_curr{ evaluate(x0) };
    double dui_cost_prev{ 0.0 };
    do
    {
      dui_cost_prev = dui_cost_curr;
      update_sequence();
      dui_cost_curr = evaluate(x0);
      // PRX_DEBUG_VAR_2(dui_cost_curr, dui_cost_prev);
    } while (std::fabs(dui_cost_curr - dui_cost_prev) > epsilon);
  }

  template <typename Controls, typename StateX>
  void update(const Controls controls, const StateX x0, const std::size_t iters)
  {
    PRX_DEBUG_VAR_1("INIT")
    init_sequence(x0, controls);
    // to_file(ofs_map, 0);
    PRX_DEBUG_VAR_1("EVALUATE0")
    evaluate(x0);
    // to_file(ofs_map, 0);
    for (int i = 0; i < iters; ++i)
    {
      PRX_DEBUG_VAR_1("UPDATE")
      update_sequence();
      PRX_DEBUG_VAR_1("EVALUATE")
      evaluate(x0);
      to_file(ofs_map, i);
    }
  }

  template <typename Controls>
  std::vector<VectorX> compute_trajectory(const VectorX x0, const Controls controls)
  {
    std::vector<VectorX> traj;
    traj.emplace_back(x0);
    for (auto ctrl : controls)
    {
      traj.emplace_back(_dynamic_function(traj.back(), ctrl));
    }
    return traj;
  }

  template <typename Controls>
  void init_sequence(const VectorX x0, const Controls controls)
  {
    // const std::size_t N{ trajectory.size() };
    const std::size_t M{ controls.size() };
    // prx_assert(N == (M + 1), "Mismatch on sizes of trajectory (" << N << ") and controls (" << M << ").");

    std::vector<VectorX> trajectory{ compute_trajectory(x0, controls) };
    _xN = trajectory[M];

    SN = _Qf;
    vN = _Qf * (_xN - _xGoal);

    VectorX xi{ trajectory[M - 1] };  // Next to last
    VectorU ui{ controls[M - 1] };    // u{N-1}' at this point

    std::size_t idx{ M };
    _sequence.emplace_front(idx, xi, ui, _dynamic_function, &SN, &_Qf, &_R, &vN);
    for (; idx > 0; --idx)
    {
      xi = trajectory[idx - 1];
      ui = controls[idx - 1];

      const MatrixS* Si{ &_sequence.front().S() };
      const VectorV* vi{ &_sequence.front().v() };

      _sequence.emplace_front(idx - 1, xi, ui, _dynamic_function, Si, &_Q, &_R, vi);
    }
  }

  void update_sequence()
  {
    std::size_t idx{ _sequence.size() - 1 };

    for (; idx > 0; --idx)
    {
      _sequence[idx].evaluate();
    }
    _sequence[idx].evaluate();  // idx = 0
  }

  // compute the optimal controls ui* = ui + dui
  double evaluate(const VectorX x0)
  {
    const std::size_t N{ _sequence.size() };
    VectorX xi{ x0 };
    VectorX dxi{ VectorX::Zero() };
    double cost{ 0.0 };
    // const std::vector<VectorX> trajectory{ compute_trajectory(x0, controls) };
    for (int i = 0; i < N; ++i)
    {
      _sequence[i].X() = xi;

      VectorU& ui{ _sequence[i].U() };
      const VectorU dui{ _sequence[i].du(dxi) };

      PRX_DEBUG_VAR_3(i, ui.transpose(), dui.transpose());
      PRX_DEBUG_VAR_3(i, xi.transpose(), dxi.transpose());
      ui = ui + dui;

      // force check for 1x1 mat
      const Eigen::Vector<double, 1> du_cost_v{ ui.transpose() * _R * ui };
      const Eigen::Vector<double, 1> dx_cost_v{ xi.transpose() * _Q * xi };
      cost += du_cost_v[0];
      cost += dx_cost_v[0];

      dxi = _sequence[i].dx_next(dxi);
      xi = _dynamic_function(xi, ui);

      PRX_DEBUG_VAR_2(i, ui.transpose());
      PRX_DEBUG_VAR_2(i, xi.transpose());
      PRX_DEBUG_VAR_2(i, cost);
    }
    const Eigen::Vector<double, 1> dx_cost_v{ dxi.transpose() * _Qf * dxi };  // force check for 1x1 mat
    cost += dx_cost_v[0];

    // _sequence[N - 1].X() = xi;
    _xN = xi;
    vN = _Qf * (_xN - _xGoal);

    return cost;
  }

  const VectorU& control(const std::size_t idx)
  {
    return _sequence[idx].U();
  }
  const VectorX& state(const std::size_t idx)
  {
    if (idx == _sequence.size())
    {
      return _xN;
    }
    return _sequence[idx].X();
  }

  void to_file(std::ofstream& ofs, std::size_t idx)
  {
    const std::size_t N{ _sequence.size() };
    for (std::size_t i = 0; i < N; ++i)
    {
      ofs << "ilqr " << idx << " ";
      ofs << state(i).transpose() << " ";
      ofs << control(i).transpose() << "\n";
    }
    ofs << "ilqr " << idx << " ";
    ofs << state(N).transpose() << "\n";
  }

private:
  MatrixQ _Qf;

  MatrixQ _Q;
  MatrixR _R;
  DynamicFunction _dynamic_function;

  VectorX _xN;
  VectorX _xGoal;

  MatrixS SN;
  VectorV vN;
  std::deque<iLQRState> _sequence;
  // std::deque<VectorU> _controls;    // ToDo: make this template to also handle prx::plan_t
  // std::deque<VectorX> _trajectory;  // ToDo: make this template to also handle prx::plan_t

  std::ofstream ofs_map;
};

}  // namespace simulation
}  // namespace prx