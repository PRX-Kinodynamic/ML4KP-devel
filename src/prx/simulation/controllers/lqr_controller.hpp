#pragma once

#include <functional>

#include "prx/simulation/controllers/lqr.hpp"
#include "prx/simulation/system_controller.hpp"
#include "prx/utilities/math/first_order_derivative.hpp"

namespace prx
{
namespace simulation
{
using namespace std::placeholders;
template <uint8_t Evaluations = 5, int8_t MinDifference = -1>
class lqr_controller_t : public system_controller_t
{
public:
  using LQR = lqr_t<Eigen::Dynamic, Eigen::Dynamic>;
  using MatrixA = LQR::MatrixA;
  using MatrixB = LQR::MatrixB;

  using MatrixQ = LQR::MatrixQ;
  using MatrixR = LQR::MatrixR;

  using VectorU = LQR::VectorU;
  using VectorX = LQR::VectorX;

  using DynamicFunctionX = std::function<VectorX(const VectorX&)>;
  using DynamicFunctionU = std::function<VectorX(const VectorU&)>;

  using DerivA = prx::math::first_order_derivative_t<DynamicFunctionX, VectorX, Evaluations, MinDifference>;
  using DerivB = prx::math::first_order_derivative_t<DynamicFunctionU, VectorU, Evaluations, MinDifference>;

  template <typename MatQ, typename MatR, typename VecX, typename VecU>
  lqr_controller_t(system_ptr_t plant, const std::string& path, MatQ q, MatR r, VecX x0, VecU u0)
    : system_controller_t(plant, path)
    , _Xdim(plant->get_state_space()->size())
    , _Udim(plant->get_control_space()->size())
    , _x0(x0)
    , _u0(u0)
    , _dynamic_function_x(std::bind(&lqr_controller_t::dynamics, this, _1, _u0))
    , _dynamic_function_u(std::bind(&lqr_controller_t::dynamics, this, _x0, _1))
    , _A_deriv(_dynamic_function_x, prx::simulation_step, _Xdim, _Xdim)
    , _B_deriv(_dynamic_function_u, prx::simulation_step, _Udim, _Xdim)
    , _lqr(_Xdim, _Udim)
    , _x(VectorX::Zero(_Xdim))
    , _x_ref(x0)
  {
    _lqr.Q() = q;
    _lqr.R() = r;
    _lqr.A() = _A_deriv(x0);
    _lqr.B() = _B_deriv(u0);
    _lqr.compute_K();
  }

  virtual ~lqr_controller_t()
  {
  }

  virtual void compute_control() override
  {
    _plant->get_state_space()->copy_to(_x);
    const VectorU ctrl{ _lqr(_x, _x_ref) };
    _plant->get_control_space()->copy_from(ctrl);
  }

  // get the underling LQR
  inline LQR& lqr()
  {
    return _lqr;
  }

protected:
  // \dot{x_t} = dynamics(x_t, u_t) --> x_{t+1} = x_t + \dot{x_t}dt
  VectorX dynamics(const VectorX& x_in, const VectorU& u_in)
  {
    _plant->get_state_space()->copy_from(x_in);
    _plant->get_control_space()->copy_from(u_in);
    _plant->propagate(prx::simulation_step * prx::simulation_step);
    _plant->get_derivative_space()->copy_to(_x);
    return _x;
  };

  const Eigen::Index _Xdim;
  const Eigen::Index _Udim;

  const VectorX _x0;
  const VectorU _u0;

  DynamicFunctionX _dynamic_function_x;
  DynamicFunctionU _dynamic_function_u;

  DerivA _A_deriv;
  DerivB _B_deriv;

  LQR _lqr;

  VectorX _x;
  VectorX _x_ref;
};
}  // namespace simulation
}  // namespace prx