import io
import os
import sys
import math
import numpy as np 
from tqdm import tqdm
import libpyDirtMP as prx

def random_point_2dtorus(): 
  rad = prx.uniform_random(5,10)
  ang = prx.uniform_random(0,2.0*prx.PRX_PI)

  x0 = rad * math.sin(ang);
  x1 = rad * math.cos(ang);
  return [x0,x1]

def random_points_s_shape():

  rad = prx.uniform_random(5,10)
  ang = prx.uniform_random(0,2.0*prx.PRX_PI)
  noise = prx.uniform_random(0, 1) # to avoid zero volume at pi

  x0 = ang;
  x1 = rad * math.sin(ang+noise);

  return [x0,x1]

if __name__ == "__main__":
  dimension = 2;
  delaunay_metric = prx.delaunay_metric.wrap(prx.default_delaunay_metric);
  dgnn = prx.delaunay_graph(delaunay_metric, dimension);
  im_map = {};


  for i in range(500):
    # pt = random_point_2dtorus()
    pt = random_points_s_shape();
    start_id = dgnn.add_point(pt);
  
  dgnn.qhull_to_delaunay();

  dgnn.to_file(prx.out_path + "latent_test.txt");    