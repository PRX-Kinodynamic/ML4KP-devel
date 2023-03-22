#pragma once

#include "libqhullcpp/RboxPoints.h"
#include "libqhullcpp/QhullError.h"
#include "libqhullcpp/QhullQh.h"
#include "libqhullcpp/QhullFacet.h"
#include "libqhullcpp/QhullFacetList.h"
#include "libqhullcpp/QhullFacetSet.h"
#include "libqhullcpp/QhullLinkedList.h"
#include "libqhullcpp/QhullPoint.h"
#include "libqhullcpp/QhullUser.h"
#include "libqhullcpp/QhullVertex.h"
#include "libqhullcpp/QhullVertexSet.h"
#include "libqhullcpp/Qhull.h"

#include "prx/utilities/data_structures/gnn.hpp"
#include "prx/utilities/defs.hpp"

namespace prx
{
namespace utilities
{
template <Eigen::Index Dim>
class delaunay_node_t : public prx::proximity_node_t<Eigen::Vector<double, Dim>>
{
public:
  using Point = Eigen::Vector<double, Dim>;
  using Sites = std::vector<Point>;
  Sites sites;
};

template <Eigen::Index Dim>
class delaunay_graph_t
{
public:
  using Point = Eigen::Vector<double, Dim>;
  using GnnNodes = std::vector<delaunay_node_t<Dim>*>;
  using DelaunayGnn = prx::graph_nearest_neighbors_t<Point, delaunay_node_t<Dim>>;
  using DelaunayMetric = typename DelaunayGnn::Metric;

  static inline double default_delaunay_metric(const Point& a, const Point& b)
  {
    return (a - b).norm();
  }

  template <Eigen::Index di = Dim, std::enable_if_t<(di != Eigen::Dynamic), bool> = true>
  delaunay_graph_t(DelaunayMetric& distance_function) : _dimension(Dim), _point_count(0)
  {
    _gnn = std::make_shared<DelaunayGnn>(distance_function);
  }
  template <Eigen::Index di = Dim, std::enable_if_t<(di == Eigen::Dynamic), bool> = true>
  delaunay_graph_t(DelaunayMetric& distance_function, const Eigen::Index dimension)
    : _dimension(dimension), _point_count(0)
  {
    _gnn = std::make_shared<DelaunayGnn>(distance_function);
    _gnn->init_query_point(Eigen::VectorXd::Zero(_dimension));
  }

  void qhull_to_delaunay()
  {
    orgQhull::Qhull qhull("", _dimension, _point_count, _points_data.data(), "d Qt");
    // The Delaunay diagram is equivalent to the convex hull of a paraboloid, one dimension higher
    int hullDimension = qhull.hullDimension();

    // Input sites as a vector of vectors
    std::vector<std::vector<double>> inputSites;
    orgQhull::QhullPoints points = qhull.points();
    // for(QhullPoint point : points)
    orgQhull::QhullPointsIterator j(points);
    while (j.hasNext())
    {
      orgQhull::QhullPoint point = j.next();
      inputSites.push_back(point.toStdVector());
    }

    // Printer header and Voronoi vertices
    orgQhull::QhullFacetList facets = qhull.facetList();

    // Delaunay regions as a vector of vectors
    std::vector<std::vector<int>> regions;
    // for(QhullFacet f : facets)
    orgQhull::QhullFacetListIterator k(facets);
    while (k.hasNext())
    {
      orgQhull::QhullFacet f = k.next();
      std::vector<int> vertices;
      if (!f.isUpperDelaunay())
      {
        if (!f.isTopOrient() && f.isSimplicial())
        { /* orient the vertices like option 'o' */
          orgQhull::QhullVertexSet vs = f.vertices();
          vertices.push_back(vs[1].point().id());
          vertices.push_back(vs[0].point().id());
          for (int i = 2; i < (int)vs.size(); ++i)
          {
            vertices.push_back(vs[i].point().id());
          }
        }
        else
        {
          orgQhull::QhullVertexSetIterator i(f.vertices());
          while (i.hasNext())
          {
            orgQhull::QhullVertex vertex = i.next();
            orgQhull::QhullPoint p = vertex.point();
            vertices.push_back(p.id());
          }
        }
        regions.push_back(vertices);
      }
    }

    for (size_t k2 = 0; k2 < regions.size(); ++k2)
    {
      std::vector<int> vertices = regions[k2];
      size_t n = vertices.size();
      Point centroid{ Point::Zero(_dimension) };
      // _gnn_nodes.push_back(std::make_shared<delaunay_node_t<Dim>>());
      _gnn_nodes.push_back(new delaunay_node_t<Dim>());
      for (size_t i = 0; i < n; ++i)
      {
        // std::vector<double> site = inputSites[vertices[i]];
        Eigen::Map<Point> site{ inputSites[vertices[i]].data(), _dimension };
        centroid += site;
        _gnn_nodes.back()->sites.emplace_back(site(Eigen::seqN(0, _dimension)));
      }
      centroid = centroid / n;
      _gnn_nodes.back()->point = Point(centroid(Eigen::seqN(0, _dimension)));
      _gnn->add_node(_gnn_nodes.back());
    }
  }

  std::shared_ptr<DelaunayGnn> get_gnn()
  {
    return _gnn;
  }

  template <typename Pt, std::enable_if_t<!prx::utils::is_ptr_type<Pt>{}, bool> = true>
  void add_point(const Pt& p)
  {
    for (auto e : p)
    {
      _points_data.push_back(e);
    }
    _point_count++;
  }

  template <typename Pt, std::enable_if_t<prx::utils::is_ptr_type<Pt>{}, bool> = true>
  void add_point(const Pt& p)
  {
    for (auto e : *p)
    {
      _points_data.push_back(e);
    }
    _point_count++;
  }

  GnnNodes get_gnn_nodes()
  {
    return _gnn_nodes;
  }

protected:
  const Eigen::Index _dimension;
  std::shared_ptr<DelaunayGnn> _gnn;
  GnnNodes _gnn_nodes;
  std::vector<double> _points_data;
  std::size_t _point_count;
  // Qhull q("", 2, pointCount, pts.data(), "d Qt");
  // Qhull qhull;
};
}  // namespace utilities
}  // namespace prx