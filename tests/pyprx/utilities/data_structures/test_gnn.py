import os
import math
import pytest
import libpyDirtMP as prx

# def gnn_builds_ok():
  # using Point = Eigen::Vector2d;
  # using Metric = std::function<double(const Point&, const Point&)>;
  # Metric metric = [](const Point& a, const Point& b) { return (a - b).norm(); };
  # prx::graph_nearest_neighbors_t<Point> gnn(metric);

  # const long unsigned expected_nr_nodes{ 0 };
  # const long unsigned resulting_nr_nodes{ gnn.get_nr_nodes() };
  # BOOST_CHECK_MESSAGE(expected_nr_nodes == resulting_nr_nodes,
  #                     "Expected: " << expected_nr_nodes << ". Got: " << resulting_nr_nodes << ".");

# if __name__ == "__main__":
  # gnn_builds_ok();