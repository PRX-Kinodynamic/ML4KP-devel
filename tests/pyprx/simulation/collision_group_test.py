import os
import math
import pytest
import PyML4KP as prx

def test_empty_environment():
  pt = prx.system_factory.create_system("2D_Point","2D_Point")
  cg = prx.collision_group([pt], []);
  assert(cg.in_collision() == False)

def test_collision():
  obstacles_names = ["Obstacle0"]
  pt = prx.system_factory.create_system("2D_Point","2D_Point")
  ss = pt.get_state_space()
  ss.copy_from([0, 0])
  
  obstacle_pose = prx.transform()
  obstacle_pose.setIdentity();
  obstacle_pose.translation(prx.vector(0.1, 0.0, 0.25))
  obstacles = prx.sphere.create(obstacles_names[0], 0.5, obstacle_pose, "0xff0000" )
  cg = prx.collision_group([pt], [obstacles]);
  result = cg.in_collision()
  assert( result )

def test_distances():
  obstacles_names = ["Obstacle0"]
  pt = prx.system_factory.create_system("2D_Point","2D_Point")
  ss = pt.get_state_space()
  ss.copy_from([0, 0])
  
  obstacle_pose = prx.transform()
  obstacle_pose.setIdentity();
  # 2d point is a cylinder of [0.5,0.5]
  obstacle_pose.translation(prx.vector(1.1, 0.0, 0.25))
  obstacles = prx.sphere.create(obstacles_names[0], 0.5, obstacle_pose, "0xff0000" )
  cg = prx.collision_group([pt], [obstacles]);
  result = cg.get_distances()
  assert( math.fabs(result.distances[0] - 0.1 ) < 0.0001)

  print(result.closest_points[0])
  x_plant = result.closest_points[0].first[0]
  y_plant = result.closest_points[0].first[1]
  z_plant = result.closest_points[0].first[2]

  x_obstacle = result.closest_points[0].second[0]
  y_obstacle = result.closest_points[0].second[1]
  z_obstacle = result.closest_points[0].second[2]
  print(result.closest_points[0].second)
  epsilon = 0.0001
  assert(math.fabs(x_plant - 0.5) < epsilon)
  assert(math.fabs(x_obstacle - 0.6) < epsilon)
  assert(math.fabs(y_plant - y_obstacle) < epsilon)
  assert(math.fabs(z_plant - z_obstacle) < epsilon)