#pragma once

#include "prx/simulation/sensor.hpp"

namespace prx
{
namespace mujoco
{
class mujoco_sensor_t : public prx::simulation::sensor_t
{
public:
  mujoco_sensor_t(mjModel* model, mjData* data, const std::size_t sensor_idx)
  {
    prx_assert(model != nullptr, "Received a nullptr model!");
    const mjtSensor_ sensor_type{ static_cast<mjtSensor_>(model->sensor_type[sensor_idx]) };
    _sensor_name = model->names + model->name_sensoradr[sensor_idx];

    std::string sensor_topology;
    int current_dim{ 0 };
    int sensor_dim = model->sensor_dim[sensor_idx];
    switch (sensor_type)
    {
      case mjSENS_TENDONPOS:
        sensor_topology += euclidean_dimension(sensor_dim, current_dim, model, data, sensor_idx);
        break;
      case mjSENS_FRAMEPOS:
        sensor_topology += euclidean_dimension(sensor_dim, current_dim, model, data, sensor_idx);
        break;
      default:
        prx_throw("Type of sensor [" << _sensor_name << "] not supported");
    }

    sensor_space = new space_t(sensor_topology, sensor_memory, _sensor_name);
  }
  virtual ~mujoco_sensor_t()
  {
  }

  virtual void sense()
  {
  }

protected:
  std::string euclidean_dimension(const int dims_to_add, const std::size_t current_dim, mjModel* model, mjData* data,
                                  const std::size_t sensor_idx)
  {
    const int address{ model->sensor_adr[sensor_idx] };
    for (int i = 0; i < dims_to_add; ++i)
    {
      sensor_memory.push_back(new double);
      sensor_memory[current_dim + i] = &data->sensordata[address + i];
    }
    return std::string(dims_to_add, 'E');
  }
};
}  // namespace mujoco
}  // namespace prx
