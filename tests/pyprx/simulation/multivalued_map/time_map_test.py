import os
import math
import pytest
import libpyDirtMP as prx

def test_time_map_pendulum_lqr_is_built_correctly():
  plant_name = "pendulum" ;
  plant = prx.system_factory.create_system(plant_name, plant_name)
  assert(plant != None);
  world_model = prx.world_model([ plant], []);
  world_model.create_context("context",  [ plant_name ], []);
  context = world_model.get_context("context");

  time_map = prx.time_map("pendulum_lqr", plant, context.system_group);

  assert time_map.x_goal != None, "x_goal should not be None";
  assert time_map.u_goal != None, "u_goal should not be None";

def test_time_map_pendulum_lqr_end_state_stays_in_zero_equilibrium():
  plant_name = "pendulum" ;
  plant = prx.system_factory.create_system(plant_name, plant_name)
  assert(plant != None);
  world_model = prx.world_model([ plant], []);
  world_model.create_context("context",  [ plant_name ], []);
  context = world_model.get_context("context");
  system_group = context.system_group;

  time_map = prx.time_map("pendulum_lqr", plant, system_group);
  time_map.set_duration(5);

  start_state_prx = system_group.get_state_space().make_point();
  end_state_prx = system_group.get_state_space().make_point();

  system_group.get_state_space().copy(start_state_prx, [0.1,0.1])
  time_map(start_state_prx, end_state_prx);
  assert len(end_state_prx)==2 and abs(end_state_prx[0]) <= 1e-3 and abs(end_state_prx[1]) <= 1e-3, print("end_state_prx: ", end_state_prx);

  start_state_pyobj = [0.1, 0.1 ];
  system_group.get_state_space().copy(end_state_prx, [-1,-1]);
  time_map(start_state_pyobj, end_state_prx);
  assert len(end_state_prx)==2 and abs(end_state_prx[0]) <= 1e-3 and abs(end_state_prx[1]) <= 1e-3, print("end_state_prx: ", end_state_prx);

  start_state_pyobj = [0.1, 0.1 ];
  end_state_pyobj = [-1,-1];
  time_map(start_state_pyobj, end_state_pyobj);
  assert len(end_state_pyobj)==2 and abs(end_state_pyobj[0]) <= 1e-3 and abs(end_state_pyobj[1]) <= 1e-3, print("end_state_pyobj: ", end_state_pyobj);

  end_state_pyobj = [-1,-1];
  system_group.get_state_space().copy(start_state_prx, [0.1,0.1])
  time_map(start_state_prx, end_state_pyobj);
  assert len(end_state_pyobj)==2 and abs(end_state_pyobj[0]) <= 1e-3 and abs(end_state_pyobj[1]) <= 1e-3, print("end_state_pyobj: ", end_state_pyobj);

if __name__ == "__main__":
  test_time_map_pendulum_lqr_is_built_correctly();
  test_time_map_pendulum_lqr_end_state_stays_in_zero_equilibrium();