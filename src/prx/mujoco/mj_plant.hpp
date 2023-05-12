#pragma once
#ifndef MUJOCO_NOT_BUILT
#include "prx/simulation/plant.hpp"

#include "prx/mujoco/mj_simulator.hpp"

namespace prx
{
// class mujoco_simulator_t;
class mujoco_plant_t : public plant_t
{
public:
  mujoco_plant_t(const std::string& path);
  virtual ~mujoco_plant_t();

  virtual void initialize(std::shared_ptr<mujoco_simulator_t> sim);

  virtual void update_configuration() override;

  virtual void compute_derivative() override;

  virtual void propagate(const double simulation_step) override final;

  virtual void compute_control() override;

  std::shared_ptr<mujoco_simulator_t> sim;
};
}  // namespace prx
#endif
