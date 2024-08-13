#pragma once

#include "prx/simulation/plant.hpp"

namespace prx
{
class delivery_robot_fo_t : public plant_t
{
public:
  delivery_robot_fo_t(const std::string& path);

  virtual ~delivery_robot_fo_t();

  virtual void propagate(const double simulation_step) override final;

  virtual void update_configuration() override;

  virtual void compute_derivative() override final;

protected:
  double x, y, theta, m, vl, vr, pick, dx, dy, dtheta, dm;
};
}  // namespace prx
PRX_REGISTER_SYSTEM(delivery_robot_fo_t, FO_delivery_robot)