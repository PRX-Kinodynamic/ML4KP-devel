#pragma once

#include "prx/simulation/plant.hpp"
#include "prx/simulation/plants/types/linear_time_variant.hpp"

#include <Eigen/Dense>
#include <Eigen/Core>

namespace prx
{
class ackermann_SO : public plant_t
{
public:
  ackermann_SO(const std::string& path);
  virtual ~ackermann_SO();

  virtual void propagate(const double simulation_step) override final;

  virtual void update_configuration() override;

protected:
  virtual void compute_derivative() override final;

  double x;
  double y;
  double theta;

  double x_dot;
  double y_dot;
  double theta_dot;

  double gamma;
  double velocity;

  double gamma_change;
  double accel;

  // Distance between front and back wheels
  double L = 1;

  const double max_delta_deg = 60;
  const double max_delta_rad = max_delta_deg * PRX_PI / 180.0;
};
}  // namespace prx

PRX_REGISTER_SYSTEM(ackermann_SO, Ackermann_SO)
