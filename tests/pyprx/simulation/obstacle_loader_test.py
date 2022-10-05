import os
import math
import pytest
import libpyDirtMP as prx

def test_empty_environment():
    obstacle_loader = prx.obstacle_loader("environments/empty.yaml");
    obstacles_names = obstacle_loader.get_names();
    obstacles = obstacle_loader.get_obstacles();

    assert(len(obstacles_names) == 0 );
    assert(len(obstacles) == 0 );

def test_simple_obstacle():
    obstacle_loader = prx.obstacle_loader("environments/simple_obstacle.yaml");
    obstacles_names = obstacle_loader.get_names();
    obstacles = obstacle_loader.get_obstacles();

    assert(obstacles_names[0] == "simple_obstacle" );
    assert(len(obstacles) == 1 );