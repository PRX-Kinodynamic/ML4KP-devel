import os
import math
import pytest
import libpyDirtMP as prx

def dijkstra_shortest_path_when_nodes_are_neighbors():
  n0_id = 0
  n1_id = 1
  points = {};
  neighbors = {};

  points[n0_id] = [0,0];
  points[n1_id] = [0,1];

  neighbors[n0_id] = [n1_id];
  neighbors[n1_id] = [n0_id];

  def neighbors_query_py(idx):
    return neighbors[idx]
  def dijkstra_distance_py(a, b):
    p_a = points[a]
    p_b = points[b]
    return math.sqrt((p_a[0]-p_b[0])**2 + (p_a[1]-p_b[1])**2)

  neighbors_query = prx.neighbors_query.wrap(neighbors_query_py)
  dijkstra_distance = prx.dijkstra_distance.wrap(dijkstra_distance_py)

  sp = prx.dijkstra.shortest_path(n0_id, n1_id, neighbors_query, dijkstra_distance);

  assert sp[0] == 0 and sp[1] ==1

def dijkstra_shortest_path_of_a_line_is_a_line():
  n0_id = 0
  n1_id = 1
  n2_id = 2
  n3_id = 3
  points = {};
  neighbors = {};

  points[n0_id] = [0,0];
  points[n1_id] = [0,1];
  points[n2_id] = [0,2];
  points[n3_id] = [0,3];

  neighbors[n0_id] = [n1_id];
  neighbors[n1_id] = [n0_id, n2_id];
  neighbors[n2_id] = [n1_id, n3_id];
  neighbors[n3_id] = [n2_id];

  def neighbors_query_py(idx):
    return neighbors[idx]
  def dijkstra_distance_py(a, b):
    p_a = points[a]
    p_b = points[b]
    return math.sqrt((p_a[0]-p_b[0])**2 + (p_a[1]-p_b[1])**2)

  neighbors_query = prx.neighbors_query.wrap(neighbors_query_py)
  dijkstra_distance = prx.dijkstra_distance.wrap(dijkstra_distance_py)

  sp = prx.dijkstra.shortest_path(n0_id, n3_id, neighbors_query, dijkstra_distance);

  assert sp[0] == 0 and sp[1] ==1 and sp[2]==2 and sp[3] == 3

def dijkstra_shortest_path_in_a_graph_with_multiple_paths():
  n0_id = 0
  n1_id = 1
  n2_id = 2
  n3_id = 3
  points = {};
  neighbors = {};

  points[n0_id] = [0,0];
  points[n1_id] = [1,-1];
  points[n2_id] = [1,2];
  points[n3_id] = [2,0];

  neighbors[n0_id] = [n1_id, n2_id];
  neighbors[n1_id] = [n0_id, n3_id];
  neighbors[n2_id] = [n0_id, n3_id];
  neighbors[n3_id] = [n1_id, n2_id];

  def neighbors_query_py(idx):
    return neighbors[idx]
  def dijkstra_distance_py(a, b):
    p_a = points[a]
    p_b = points[b]
    return math.sqrt((p_a[0]-p_b[0])**2 + (p_a[1]-p_b[1])**2)

  neighbors_query = prx.neighbors_query.wrap(neighbors_query_py)
  dijkstra_distance = prx.dijkstra_distance.wrap(dijkstra_distance_py)

  sp = prx.dijkstra.shortest_path(n0_id, n3_id, neighbors_query, dijkstra_distance);

  assert sp[0] == 0 and sp[1] ==1 and sp[2] == 3

if __name__ == "__main__":
  dijkstra_shortest_path_when_nodes_are_neighbors();
  dijkstra_shortest_path_of_a_line_is_a_line()
  dijkstra_shortest_path_in_a_graph_with_multiple_paths()