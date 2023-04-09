#pragma once
#ifndef QHULL_NOT_BUILT

#include <Eigen/Dense>

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
#include "prx/utilities/general/transforms.hpp"
#include <unordered_set>
namespace prx
{
namespace utilities
{

template <Eigen::Index Dim>
class delaunay_node_t : public prx::proximity_node_t<Eigen::Vector<double, Dim>>
{
public:
  using Point = Eigen::Vector<double, Dim>;
  using Neighbors = std::unordered_set<std::size_t>;
  delaunay_node_t() : proximity_node_t<Point>(), neighbors()
  {
  }

  Neighbors neighbors;
  Point image;
  std::size_t id;
};

template <Eigen::Index Dim>
class delaunay_graph_t
{
public:
  using Point = Eigen::Vector<double, Dim>;
  using Node = delaunay_node_t<Dim>;
  using NodePtr = Node*;
  using NodeMap = std::unordered_map<std::size_t, NodePtr>;
  using DelaunayGnn = prx::graph_nearest_neighbors_t<Point, delaunay_node_t<Dim>>;
  using DelaunayMetric = typename DelaunayGnn::Metric;

  static inline double default_delaunay_metric(const Point& a, const Point& b)
  {
    return (a - b).norm();
  }

  class iterator
  {
    typename NodeMap::iterator _iter;

  public:
    using iterator_category = std::output_iterator_tag;
    using value_type = NodePtr;  // crap
    using difference_type = NodePtr;
    using pointer = const NodePtr*;
    using reference = NodePtr;

    explicit iterator(typename NodeMap::iterator iter) : _iter(iter)
    {
    }

    iterator& operator++()
    {
      _iter++;
      return *this;
    }
    iterator operator++(int)
    {
      iterator retval = *this;
      ++(*this);
      return retval;
    }
    bool operator==(iterator other) const
    {
      return _iter == other._iter;
    }
    bool operator!=(iterator other) const
    {
      return !(*this == other);
    }
    reference operator*() const
    {
      return (*_iter).second;
    }
  };
  iterator begin()
  {
    return iterator(_nodes.begin());
  }
  iterator end()
  {
    return iterator(_nodes.end());
  }

  template <Eigen::Index di = Dim, std::enable_if_t<(di != Eigen::Dynamic), bool> = true>
  delaunay_graph_t(DelaunayMetric& distance_function) : _dimension(Dim), _point_count(0)
  {
    _gnn = std::make_shared<DelaunayGnn>(distance_function);
  }
  template <Eigen::Index di = Dim, std::enable_if_t<(di == Eigen::Dynamic), bool> = true>
  delaunay_graph_t(DelaunayMetric& distance_function, Eigen::Index dimension) : _dimension(dimension), _point_count(0)
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
    // std::vector<std::vector<double>> inputSites;
    // orgQhull::QhullPoints points = qhull.points();
    // for(QhullPoint point : points)
    // orgQhull::QhullPointsIterator j(points);
    // while (j.hasNext())
    // {
    //   orgQhull::QhullPoint point = j.next();
    //   PRX_DEBUG_VAR_2(point.id(), point);
    //   //   inputSites.push_back(point.toStdVector());
    // }

    // Printer header and Voronoi vertices
    orgQhull::QhullFacetList facets = qhull.facetList();

    // Delaunay regions as a vector of vectors
    std::vector<std::vector<int>> regions;
    // for(QhullFacet f : facets)
    orgQhull::QhullFacetListIterator k(facets);
    NodePtr current_node;
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
            // const Eigen::Map<Point> site{ p.coordinates(), _dimension };
            // const std::size_t v_id{ _hash(site) };
            // current_node = _nodes[v_id];
            // current_node->neighbors.insert(vertices.begin(), iter);

            vertices.push_back(p.id());
          }
        }
        for (auto iter = vertices.begin(); iter != vertices.end(); iter++)
        {
          const std::size_t v_id{ static_cast<std::size_t>(*iter) };
          current_node = _nodes[v_id];

          if (iter != vertices.begin())
          {
            current_node->neighbors.insert(vertices.begin(), iter);
          }
          current_node->neighbors.insert(iter + 1, vertices.end());
        }
        // regions.push_back(vertices);
      }
    }

    // for (size_t k2 = 0; k2 < regions.size(); ++k2)
    // {
    //   std::vector<int> vertices = regions[k2];
    //   size_t n = vertices.size();
    //   Point centroid{ Point::Zero(_dimension) };
    //   // _gnn_nodes.push_back(std::make_shared<delaunay_node_t<Dim>>());
    //   _gnn_nodes.push_back(new delaunay_node_t<Dim>());
    //   for (size_t i = 0; i < n; ++i)
    //   {
    //     // std::vector<double> site = inputSites[vertices[i]];
    //     Eigen::Map<Point> site{ inputSites[vertices[i]].data(), _dimension };
    //     centroid += site;
    //     _gnn_nodes.back()->sites.emplace_back(site(Eigen::seqN(0, _dimension)));
    //   }
    //   centroid = centroid / n;
    //   _gnn_nodes.back()->point = Point(centroid(Eigen::seqN(0, _dimension)));
    //   _gnn->add_node(_gnn_nodes.back());
    // }
  }

  std::shared_ptr<DelaunayGnn> get_gnn()
  {
    return _gnn;
  }

  template <typename Pt, std::enable_if_t<!prx::utils::is_ptr_type<Pt>{}, bool> = true>
  std::size_t add_point(const Pt& p)
  {
    std::size_t i{ 0 };
    Point pt{ Point::Zero(_dimension) };
    for (std::size_t i = 0; i < p.size(); ++i)
    {
      auto e = p[i];
      _points_data.push_back(e);
      pt[i] = e;
      i++;
    }
    NodePtr new_node = new Node();
    new_node->id = _point_count;
    new_node->point = pt;
    _nodes[_point_count] = new_node;
    _point_count++;
    return new_node->id;
  }

  template <typename Pt, std::enable_if_t<prx::utils::is_ptr_type<Pt>{}, bool> = true>
  std::size_t add_point(const Pt& p)
  {
    return add_point(*p);
  }

  const inline NodePtr operator[](const std::size_t& idx)
  {
    return _nodes[idx];
  }

  inline NodePtr& at(const std::size_t& idx)
  {
    return _nodes[idx];
  }

  const NodeMap get_nodes()
  {
    return _nodes;
  }

  std::vector<NodePtr> get_neighbors(const NodePtr node) const
  {
    std::vector<NodePtr> result_neighbors;
    for (auto neighbor : node->neighbors)
    {
      result_neighbors.push_back(_nodes[neighbor]);
    }
    return result_neighbors;
  }
  inline std::vector<NodePtr> get_neighbors(const std::size_t idx) const
  {
    return get_neighbors(_nodes[idx]);
  }

  void to_file(const std::string filename, const std::ios_base::openmode _mode = std::ofstream::trunc)
  {
    std::ofstream ofs_sites;
    ofs_sites.open(filename.c_str(), _mode);

    auto delaunay_nodes = _nodes;
    for (auto node : delaunay_nodes)
    {
      for (auto neighbor : node.second->neighbors)
      {
        auto site = _nodes[neighbor];
        prx_assert(site != nullptr, "Site is nullptr");
        ofs_sites << node.second->point.transpose() << " ";
        ofs_sites << site->point.transpose() << " ";
        ofs_sites << "\n";
      }
    }
  }

  inline std::size_t size() const
  {
    return _nodes.size();
  }

protected:
  const Eigen::Index _dimension;
  std::shared_ptr<DelaunayGnn> _gnn;
  NodeMap _nodes;
  std::vector<double> _points_data;
  std::size_t _point_count;

  range_hash_combine_t<Point, Dim> _hash;
  // Qhull q("", 2, pointCount, pts.data(), "d Qt");
  // Qhull qhull;
};
}  // namespace utilities
}  // namespace prx

#endif
