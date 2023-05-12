#pragma once

#include "prx/simulation/sensor.hpp"

namespace prx
{
namespace mujoco
{
class mujoco_site_t
{
  mujoco_site_t(mjModel* model, mjData* data, const std::size_t site_idx)
  {
    prx_assert(model != nullptr, "Received a nullptr model!");
    _site_name = model->names + model->name_siteadr[sensor_idx];

    sensor_memory[current_dim] = &data->sensordata[address + i];

    const int address{ model->sensor_adr[sensor_idx] };
    // _site_transform.translation() = &data->site_pos[];
    // _site_transform.linear() = &data->site_quat[];
  }

  virtual ~mujoco_site_t()
  {
  }

protected:
  std::string _site_name;
  Eigen::Transform<double, 3, Eigen::Isometry> _site_transform;
}  // namespace mujoco
}  // namespace prx