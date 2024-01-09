import math
import pytest
import PyML4KP as prx

def metric(a, b):
  distance = 0;
  for ai,bi in zip(a,b):
    distance += (ai-bi)**2
  return math.sqrt(distance)

def get_mock_space():
  smp = prx.space_memory(2)
  space = prx.space_t("EE", smp, "space_1");

  return space;

def test_gnn_builds_ok():
  df = prx.distance_function.wrap(metric)
  gnn = prx.graph_nearest_neighbors(df);

  expected_nr_nodes =  0 
  resulting_nr_nodes =  gnn.get_nr_nodes() 
  assert (expected_nr_nodes == resulting_nr_nodes);


def test_gnn_2d_single_query_test():
  df = prx.distance_function.wrap(metric)
  gnn = prx.graph_nearest_neighbors(df);

  space = get_mock_space()

  ## Testing case:
  ## 
  ## p0 ---- p1 --p2
  ## 
  p0 = prx.abstract_node();
  p1 = prx.abstract_node();
  p2 = prx.abstract_node();
  print(space.get_dimension())
  p0.point = space.make_point([0.0, 0.0]);
  p1.point = space.make_point([4.0, 0.0]);
  p2.point = space.make_point([6.0, 0.0]);

  gnn.add_node(p0);
  gnn.add_node(p1);
  gnn.add_node(p2);

  expected_nr_nodes =  3 
  resulting_nr_nodes =  gnn.get_nr_nodes() 
  assert(expected_nr_nodes == resulting_nr_nodes);

  p0_query =  space.make_point([1.0, 0.0]) ;
  p1_query =  space.make_point([4.5, 0.0]) ;
  p2_query =  space.make_point([6.5, 0.0]) ;

  result_closest_to_p0 = gnn.single_query(p0_query).point
  result_closest_to_p1 = gnn.single_query(p1_query).point
  result_closest_to_p2 = gnn.single_query(p2_query).point

  expected_closest_to_p0 = p0.point 
  expected_closest_to_p1 = p1.point 
  expected_closest_to_p2 = p2.point 

  assert(space.equal_points(result_closest_to_p0, expected_closest_to_p0))
                   

  assert(space.equal_points(result_closest_to_p1, expected_closest_to_p1))
                  

  assert(space.equal_points(result_closest_to_p2, expected_closest_to_p2))
                   
def test_gnn_2d_multi_query_test():
  
  df = prx.distance_function.wrap(metric)
  gnn = prx.graph_nearest_neighbors(df);

  space = get_mock_space()

  nodes =[]
  for e in range(10):
    nodes.append(prx.abstract_node())
    nodes[-1].point = space.make_point( [e,e]);
    gnn.add_node(nodes[-1]);

  query = space.make_point([ 0.25, 0.25 ]);
  expected_vector_size = 3 ;
  result_closest =  gnn.multi_query(query, expected_vector_size) ;

  assert(expected_vector_size == len(result_closest))

  result_close_0 = result_closest[0].point 
  result_close_1 = result_closest[1].point 
  result_close_2 = result_closest[2].point 

  expected_closest_to_p0 = space.make_point([ 0.0, 0.0 ]);
  expected_closest_to_p1 = space.make_point([ 1.0, 1.0 ]);
  expected_closest_to_p2 = space.make_point([ 2.0, 2.0 ]);

  assert(space.equal_points(result_close_0, expected_closest_to_p0));

  assert(space.equal_points(result_close_1, expected_closest_to_p1));

  assert(space.equal_points(result_close_2, expected_closest_to_p2));


def test_gnn_2d_radius_and_closest_query_test():
  df = prx.distance_function.wrap(metric)
  gnn = prx.graph_nearest_neighbors(df);

  space = get_mock_space()

  nodes = [];
  for e in range(10):
    nodes.append( prx.abstract_node());
    initial_value = [e,e];
    nodes[-1].point = space.make_point(initial_value);
    gnn.add_node(nodes[-1]);

  query = space.make_point([ 5.1, 5.1 ]);
  query_radius = 2.0;  # more than sqrt(2), less than 2sqrt(2)
  result_closest = gnn.radius_and_closest_query(query, query_radius) ;

  expected_vector_size = 3 ;
  assert(expected_vector_size == len(result_closest))

  result_close_0 = result_closest[0].point;
  result_close_1 = result_closest[1].point;
  result_close_2 = result_closest[2].point;

  expected_closest_to_p0 = space.make_point([5.0, 5.0]);
  expected_closest_to_p1 = space.make_point([6.0, 6.0]);
  expected_closest_to_p2 = space.make_point([4.0, 4.0]);

  assert(space.equal_points(result_close_0, expected_closest_to_p0))

  assert(space.equal_points(result_close_1, expected_closest_to_p1))

  assert(space.equal_points(result_close_2, expected_closest_to_p2))
