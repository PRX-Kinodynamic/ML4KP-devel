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
  im_map = {};

  point_count = 0;
  is_there_next = True
  result_traj = prx.trajectory(ss)

  # Code for start-end trajectories
  for i in range(100):
    ss.sample(random_state);
    ss.copy(start_state, random_state)
    time_map(start_state, result_traj);
    start_id = dgnn.add_point(result_traj[0]);
    for j in np.arange(0.1,1,0.05):
      state = result_traj.interpolate(j)
      image_id = dgnn.add_point(state);
      im_map[start_id] = image_id;
      start_id = image_id
  
  # Code for start-end states
  # while(is_there_next):
  #   ss.sample(random_state);
  #   ss.copy(start_state, random_state)
  #   # ss.copy(start_state, state)
  #   time_map(start_state, result_state);
  #   start_id = dgnn.add_point(start_state);
  #   image_id = dgnn.add_point(result_state);
  #   im_map[start_id] = image_id;

  #   is_there_next=prx.state_space_step(state, 0.5, dimension, lower_bounds, upper_bounds);

  dgnn.qhull_to_delaunay();

  ofs_ss = open((prx.out_path + "py_delaunay_start_states.txt"), "w");
  ofs_im = open((prx.out_path + "py_delaunay_end_states.txt"), "w");
  ofs_edges = open((prx.out_path + "py_delaunay_edges.txt"), "w");

  for start_id in im_map:
    image_id = im_map[start_id];
    ofs_ss.write(str(start_id) + " " + str(dgnn[start_id].point.transpose())+ "\n")
    ofs_im.write(str(image_id) + " " + str(dgnn[image_id].point.transpose())+ "\n")
    
  for node in dgnn:
    for neighbor in node.neighbors:
      ofs_edges.write(str(node.point.transpose()) + " ")
      ofs_edges.write(str(dgnn[neighbor].point.transpose()) + " ")
      ofs_edges.write("\n")

  def neighbors_query_py(idx):
    return dgnn[idx].neighbors
  def dijkstra_distance_py(a, b):
    return (dgnn[a].point - dgnn[b].point).norm();
  neighbors_query = prx.neighbors_query.wrap(neighbors_query_py)
  dijkstra_distance = prx.dijkstra_distance.wrap(dijkstra_distance_py)

  F = {};

  for v_idx in im_map:
    v_im = im_map[v_idx]
    F[v_idx] = {v_im}
    N = dgnn[v_idx].neighbors;
    for n in N:
      if n in im_map:
        y = im_map[n] ;
        sp = prx.dijkstra.shortest_path(v_im, y, neighbors_query, dijkstra_distance);
        for s in sp:
          F[v_idx].add(s)

  sites_filename = prx.out_path + "py_delaunay_ss_f.txt";
  voronoi_filename = prx.out_path + "py_delaunay_es_f.txt";
  ofs_sites = open(sites_filename, "w");
  ofs_voronoi = open(voronoi_filename, "w");

  v_idx = int(param["v_query"])
  ofs_sites.write(str(v_idx) + " " + str(dgnn[v_idx].point.transpose()) +"\n");
  for idx in dgnn[v_idx].neighbors:
    ofs_sites.write(str(idx) + " " + str(dgnn[idx].point.transpose()) +"\n");
  for idx in F[v_idx]:
    ofs_voronoi.write(str(idx) + " " + str(dgnn[idx].point.transpose()) +"\n");


