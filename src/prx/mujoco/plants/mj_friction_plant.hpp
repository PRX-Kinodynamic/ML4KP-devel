#pragma once
#ifndef MUJOCO_NOT_BUILT
#include "prx/simulation/plant.hpp"
#include "prx/mujoco/mj_plant.hpp"
#include "prx/mujoco/mj_simulator.hpp"

namespace prx
{
namespace mujoco
{
struct mj_friction_plant_t : public prx::mujoco_plant_t
{
  static void add_friction_to_plant(std::shared_ptr<prx::mujoco_simulator_t> sim, system_ptr_t plant)
  {
    std::shared_ptr<prx::mujoco_plant_t> mj_plant = std::dynamic_pointer_cast<prx::mujoco_plant_t>(plant);

    prx_assert(sim != nullptr, "Not a mujoco plant");

    const int total_geoms{ sim->_mj_model->ngeom };
    std::vector<double> ps_lb, ps_ub;
    int floor_id{ -1 };

    for (int i = 0; i < total_geoms; ++i)
    {
      std::string g1 = std::string(sim->_mj_model->names + sim->_mj_model->name_geomadr[i]);
      if (g1 == "floor0")
        floor_id = i;
    }
    prx_assert(floor_id != -1, "Floor name not found!");
    // parameter_memory.push_back(new double);
    std::string parameter_topo_string = "E";
    std::string parameter_space_name = "friction_floor";
    mj_plant->get_parameter_memory().push_back(&sim->_mj_model->geom_friction[floor_id + 2]);
    space_t* parameter_space = mj_plant->get_parameter_space();

    ps_lb = parameter_space->get_lower_bounds();
    ps_ub = parameter_space->get_upper_bounds();
    ps_lb.push_back(0.0);
    ps_ub.push_back(1.0);
    parameter_space->push_back(parameter_topo_string, mj_plant->get_parameter_memory(), parameter_space_name);
    // parameter_space = new space_t();

    parameter_space->set_bounds(ps_lb, ps_ub);
  }
};
}  // namespace mujoco
}  // namespace prx
#endif
