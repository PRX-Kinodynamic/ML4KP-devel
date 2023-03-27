import io
import os
import sys
import math
import numpy as np 
from tqdm import tqdm
import libpyDirtMP as prx

if __name__ == "__main__":

  params = prx.param_loader("examples/tripods/compute_roa.yaml", sys.argv);

  simulation_step = params["simulation_step"].as_float()
  prx.set_simulation_step(simulation_step)
  prx.init_random(params["random_seed"].as_int())

  system_name = str(params["system_name"]);

  plant_name = params["/plant/name"].as_string()
  plant_path = params["/plant/path"].as_string()
  plant = prx.system_factory.create_system(plant_name, plant_path)
  assert plant != None, "Error: plant not found!";
  
  world_model = prx.world_model([plant], []);
  world_model.create_context("context", [plant_name], []);
  context = world_model.get_context("context");

  sg = context.system_group;
  ss = sg.get_state_space();
  dimension = ss.get_dimension();

  lower_bounds = params["/plant/state_space_lower_bound"].as_float_vector()
  upper_bounds = params["/plant/state_space_upper_bound"].as_float_vector()
  ss.set_bounds(lower_bounds, upper_bounds)

  traj = prx.trajectory(ss);
  pts = [];

  tm = prx.time_map(system_name, plant, sg);
  tm.set_duration(0.5);

  state = ss.make_point();
  random_state = ss.make_point();
  start_state = ss.make_point();
  ss.copy(state, lower_bounds);
  pointCount = 0 ;
  traj_count = 0 ;

  dimension = 2;
  delaunay_metric = prx.delaunay_metric.wrap(prx.default_delaunay_metric);
  dgnn = prx.delaunay_graph(delaunay_metric, dimension);
  dgnn_im = prx.delaunay_graph(delaunay_metric, dimension);

  result_state = ss.make_point();
  qhull_data = prx.vector_of_doubles();

  point_count = 0;
  while(prx.state_space_step(state, [0.5]*dimension, dimension, lower_bounds, upper_bounds)):
    ss.copy(start_state, state)
    dgnn.add_point(start_state);
    point_count += 1
    # ss.sample(random_state);
    # ss.copy(start_state, random_state)
    # ss.copy_point(start_state, random_state)

    # tm(start_state, result_state);
    # dgnn.add_point(result_state);
  dgnn.qhull_to_delaunay();
  for pair in dgnn.get_nodes():
    tm(pair.node.point, result_state);
    print(result_state)
    dgnn_im.add_point(result_state);
  dgnn_im.qhull_to_delaunay();
  dgnn.to_file(prx.out_path + "delaunay_start_states.txt");
  dgnn_im.to_file(prx.out_path + "delaunay_end_states.txt");

  # sites_filename = open(prx.out_path + "delaunay_py.txt", 'w');
  # voronoi_filename = open(prx.out_path + "voronoi_py.txt", 'w');

  # query_node = ss.make_point();
  # ss.copy(query_node, [0.0, 0.0]);
  # # result_nodes = dgnn.get_gnn().radius_and_closest_query(query_node, 1.0);
  # result_nodes = dgnn.get_gnn_nodes();
  # for node in result_nodes:
  #   pt_str = str(node.point[0]) + " " + str(node.point[1]) + "\n" ;
  #   voronoi_filename.write(pt_str)
  #   for site in node.sites:
  #     site_str = str(site[0]) + " " + str(site[1]) + " ";
  #     # print(site_str)
  #     sites_filename.write(site_str)
  #   sites_filename.write("\n")




   
