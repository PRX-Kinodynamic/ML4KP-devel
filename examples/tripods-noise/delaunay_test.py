import io
import os
import sys
import math
import numpy as np 
from tqdm import tqdm
import libpyDirtMP as prx

if __name__ == "__main__":

  param = prx.param_loader("examples/tripods/compute_roa.yaml", sys.argv);
  time_map = prx.time_map(param);
  
  tm_data = time_map.data;
  ss = tm_data.state_space

  state = ss.make_point();
  random_state = ss.make_point();
  start_state = ss.make_point();
  result_state = ss.make_point();

  lower_bounds = param["/plant/state_space_lower_bound"].as_float_vector();
  upper_bounds = param["/plant/state_space_upper_bound"].as_float_vector();
  ss.copy(state, lower_bounds);
  pointCount = 0 ;
  traj_count = 0 ;

  dimension = 2;
  delaunay_metric = prx.delaunay_metric.wrap(prx.default_delaunay_metric);
  dgnn = prx.delaunay_graph(delaunay_metric, dimension);
  dgnn_im = prx.delaunay_graph(delaunay_metric, dimension);

  point_count = 0;
  is_there_next = True
  while(is_there_next):
    ss.sample(random_state);
    ss.copy(start_state, random_state)
    # ss.copy(start_state, state)
    time_map(start_state, result_state);
    dgnn.add_point(start_state);
    dgnn_im.add_point(result_state);

    point_count += 1
    is_there_next=prx.state_space_step(state, 0.5, dimension, lower_bounds, upper_bounds);

  dgnn.qhull_to_delaunay();
  dgnn_im.qhull_to_delaunay();
  dgnn.to_file(prx.out_path + "py_delaunay_start_states.txt");
  dgnn_im.to_file(prx.out_path + "py_delaunay_end_states.txt");

  def neighbors_query_py(idx):
    return dgnn_im[idx].neighbors
  def dijkstra_distance_py(a, b):
    return (dgnn_im[a].point - dgnn_im[b].point).norm();
  neighbors_query = prx.neighbors_query.wrap(neighbors_query_py)
  dijkstra_distance = prx.dijkstra_distance.wrap(dijkstra_distance_py)

  F = {};
  # v_idx = 87

  for node in dgnn:
    v_idx = node.id
    F[v_idx] = {v_idx}
    N = dgnn[v_idx].neighbors;
    Y = set();
    for n in N:
      Y.add(dgnn_im[n].id)

    F[v_idx] |= Y;

    for y in Y:
      sp = prx.dijkstra.shortest_path(v_idx, y, neighbors_query, dijkstra_distance);
      for s in sp:
        F[v_idx].add(s)

  sites_filename = prx.out_path + "py_delaunay_ss_f.txt";
  voronoi_filename = prx.out_path + "py_delaunay_es_f.txt";
  ofs_sites = open(sites_filename, "w");
  ofs_voronoi = open(voronoi_filename, "w");

  v_idx = 85
  ofs_sites.write(str(v_idx) + " " + str(dgnn[v_idx].point.transpose()) +"\n");
  for idx in dgnn[v_idx].neighbors:
    ofs_sites.write(str(idx) + " " + str(dgnn[idx].point.transpose()) +"\n");
  for idx in F[v_idx]:
    ofs_voronoi.write(str(idx) + " " + str(dgnn_im[idx].point.transpose()) +"\n");


