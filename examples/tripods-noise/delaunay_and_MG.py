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
  N=80
  ARANGE = np.arange(0.1,1,0.1)
  att1 = np.array([-2.12,0])
  att2 = np.array([2.12,0])
  count_points_in_attractors = 0

  for i in range(N):
    ss.sample(random_state);
    ss.copy(start_state, random_state)
    time_map(start_state, result_traj);
    start_id = dgnn.add_point(result_traj[0]);
    # for j in np.arange(0.1,1,0.05):
    for j in ARANGE:
      state = result_traj.interpolate(j)
      state_array = np.array(state.to_list())
      # aa, bb, cc = (np.linalg.norm(state_array - att1) < 0.5), (np.linalg.norm(state_array - att2) < 0.5), (state.vector().norm() < 0.5)
      # # dd = any()
      if any([(np.linalg.norm(state_array - att1) < 0.5), (np.linalg.norm(state_array - att2) < 0.5), (state.vector().norm() < 0.5)]):
        if count_points_in_attractors > 10:
          break
        count_points_in_attractors += 1

      image_id = dgnn.add_point(state);
      im_map[start_id] = image_id;
      start_id = image_id

  number_of_points = N * (len(ARANGE) + 1) # fix this later
  


  
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
    # ofs_edges.write(str(node.id) + " ")
    ofs_edges.write(str(node.point.transpose()) + " ")
    for neighbor in node.neighbors:
      # ofs_edges.write(str(neighbor) + " ")
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
