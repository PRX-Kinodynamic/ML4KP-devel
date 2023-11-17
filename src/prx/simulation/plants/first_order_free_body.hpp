#pragma once

#include "prx/simulation/plant.hpp"
#include "prx/simulation/plants/types/linear_time_variant.hpp"

namespace prx
{
class first_order_free_body_t : public plant_t
{
  using Position = Eigen::Vector3d;
  using Quaternion = Eigen::Quaternion<double>;

  using Velocity = Eigen::Vector3d;

public:
  first_order_free_body_t(const std::string& path);
  virtual ~first_order_free_body_t();

  virtual void propagate(const double simulation_step) override final;

  virtual void update_configuration() override;

protected:
  virtual void compute_derivative() override final;

  // _omega into _quat_omega: \omega -> [0, \omega]
  inline void omega_to_quaternion()
  {
    _quat_omega.w() = 0;
    _quat_omega.vec() = _omega;
  }

  Position _x;
  Quaternion _quat;

  Velocity _xdot;
  Velocity _omega;

  // Auxiliary
  Quaternion _quatdot;
  Quaternion _quat_omega;  // [0, \omega]

  double _width;
  double _depth;
  double _length;
};
}  // namespace prx

PRX_REGISTER_SYSTEM(first_order_free_body_t, first_order_free_body)
