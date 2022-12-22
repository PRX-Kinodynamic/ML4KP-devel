#pragma once

#include "prx/utilities/defs.hpp"
#include "prx/simulation/system.hpp"
// #include "prx/simulation/simulator.hpp"
#include "prx/simulation/controller.hpp"
#include "prx/planning/condition_check.hpp"
#include "prx/simulation/playback/plan.hpp"
#include "prx/simulation/playback/trajectory.hpp"
#include "prx/simulation/collision_checking/collision_checker.hpp"
namespace prx
{
class simulator_t;
class system_group_manager_t;

class noisy_system_group_t
{
}