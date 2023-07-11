#ifndef BULLET_NOT_BUILT

#include "prx/bullet_sim/plants/bullet_plant.hpp"
#include <math.h>

namespace prx
{

bullet_plant_t::bullet_plant_t(const std::string& path) : plant_t(path)
{
  /*
  Initialize the state and control spaces for the robot.
  */
  system_type = plant_type::BULLET;
}

bullet_plant_t::~bullet_plant_t()
{
}

void bullet_plant_t::propagate(const double simulation_step, const propagate_step step)
{
  prx_assert(false, "Bullet plant does not support propagate()!");
}

void bullet_plant_t::compute_control()
{
  /*
  Needs to be implemented individually for each robot.
  */
}

void bullet_plant_t::purge_saved_states()
{
  std::cout << "Purging all non inital states..." << std::endl;
  for (int i = 1; i < lastSavedId; i++)
  {
    sim->removeStateFromMemory(i);
  }
  lastSavedId = 0;
  std::cout << "Finished purging." << std::endl;
}

void bullet_plant_t::update_to_bullet(const space_point_t& point)
{
  // @aravind: I think there should be an easier way to do this.
  std::vector<double> current_state_vec;
  state_space->copy_vector_from_point(current_state_vec, point);
  sim->restoreStateFromMemory(current_state_vec.back());
}

void bullet_plant_t::update_configuration()
{
  // It doesn't make sense to update the configuration of a bullet plant.
}

void bullet_plant_t::compute_derivative()
{
}

void bullet_plant_t::setBasePositionAndRotation(btVector3 basePosition, btVector3 baseRotation)
{
  btQuaternion baseOrientation;
  bullet_simulator_t::get_quaternion_from_euler(baseOrientation, baseRotation);
  sim->resetBasePositionAndOrientation(uniqueId, basePosition, baseOrientation);
}
}  // namespace prx
#endif
