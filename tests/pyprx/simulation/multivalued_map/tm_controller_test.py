import os
import math
import pytest
import libpyDirtMP as prx

def test_time_map_controllers_check_available_systems():
  tm_controllers_names = ["pendulum_lqr"];
  available_systems = prx.time_map_controllers.available_systems() ;

  for name in tm_controllers_names:
    assert name in available_systems
  
if __name__ == "__main__":
  test_time_map_controllers_check_available_systems();