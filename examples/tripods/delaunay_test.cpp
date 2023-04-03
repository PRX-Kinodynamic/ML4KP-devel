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

#include "prx/planning/condition_check.hpp"
#include "prx/planning/planner_functions/planner_functions.hpp"
#include "prx/planning/noisy_world_model.hpp"

#include "prx/simulation/controllers/lqr.hpp"
#include "prx/simulation/controllers/noisy_controller.hpp"
#include "prx/simulation/controllers/torch_controller.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/simulation/plants/types/noisy_plant.hpp"
#include "prx/simulation/multivalued_map/systems.hpp"
#include "prx/simulation/world_model.hpp"

#include "prx/utilities/data_structures/gnn.hpp"
#include "prx/utilities/data_structures/tree.hpp"
#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/noise.hpp"
#include "prx/utilities/general/param_loader.hpp"

#include "prx/utilities/data_structures/delaunay.hpp"
#include "prx/utilities/general/dijkstra.hpp"

#include <cstdio>   /* for printf() of help message */
#include <iomanip>  // setw
#include <ostream>
#include <stdexcept>
#include <queue>

using std::cerr;
using std::cin;
using std::cout;
using std::endl;

using orgQhull::Qhull;
using orgQhull::QhullError;
using orgQhull::QhullFacet;
using orgQhull::QhullFacetList;
using orgQhull::QhullFacetListIterator;
using orgQhull::QhullFacetSet;
using orgQhull::QhullFacetSetIterator;
using orgQhull::QhullPoint;
using orgQhull::QhullPoints;
using orgQhull::QhullPointsIterator;
using orgQhull::QhullQh;
using orgQhull::QhullUser;
using orgQhull::QhullVertex;
using orgQhull::QhullVertexList;
using orgQhull::QhullVertexListIterator;
using orgQhull::QhullVertexSet;
using orgQhull::QhullVertexSetIterator;
using orgQhull::RboxPoints;

struct delaunay_node_t : prx::proximity_node_t<Eigen::Vector2d>
{
  std::vector<Eigen::Vector2d> sites;
};
using GnnNodes = std::vector<std::shared_ptr<delaunay_node_t>>;
using prx::utilities::dijkstra_t;

int main(int argc, char** argv);
int user_eg3(int argc, char** argv);
// void qdelaunay_o(const Qhull& qhull);
void qdelaunay_o(const Qhull& qhull, prx::graph_nearest_neighbors_t<Eigen::Vector2d, delaunay_node_t>& gnn,
                 GnnNodes& gnn_nodes);

char prompt[] = "";

void trajs_to_qhull();

int main(int argc, char** argv)
{
  QHULL_LIB_CHECK

  prx::param_loader params = prx::param_loader("examples/tripods/compute_roa.yaml", argc, argv);
  // params.print();

  prx::simulation_step = params["simulation_step"].as<double>();
  prx::init_random(params["random_seed"].as<int>());
  const std::string system_name{ params["system_name"].as<>() };

  const double step_inc{ params["state_increment"].as<double>() };

  auto lower_bounds = params["/plant/starting_lower_bound"].as<std::vector<double>>();
  auto upper_bounds = params["/plant/ending_upper_bound"].as<std::vector<double>>();

  std::string plant_name = params["/plant/name"].as<>();
  std::string plant_path = params["/plant/path"].as<>();
  auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
  prx_assert(plant != nullptr, "Plant is nullptr!");

  prx::world_model_t* world_model = new prx::world_model_t({ plant }, {});
  world_model->create_context("context", { plant_name }, {});
  auto context = world_model->get_context("context");

  auto sg = context.first;
  prx::space_t* ss = sg->get_state_space();
  std::size_t dimension{ ss->get_dimension() };

  prx::simulation::time_map_controllers_t::print_systems();
  // std::vector<prx::trajectory_t> trajs;
  prx::trajectory_t traj(sg->get_state_space());
  std::vector<double> pts;
  prx::simulation::time_map_t tm(system_name, plant, sg);
  tm.set_duration(0.5);

  prx::space_point_t state = ss->make_point();
  ss->copy(state, lower_bounds);
  int pointCount{ 0 };
  int traj_count{ 0 };

  GnnNodes gnn_nodes;

  prx::graph_nearest_neighbors_t<Eigen::Vector2d>::Metric df = [](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
    return (a - b).norm();
  };
  // prx::graph_nearest_neighbors_t<Eigen::Vector2d, delaunay_node_t> gnn(df, 1e5);
  using DelaunayGraph = prx::utilities::delaunay_graph_t<2>;
  using DelaunayNodePtr = DelaunayGraph::NodePtr;
  DelaunayGraph dgnn(df);
  // DelaunayGraph dgnn_im(df);

  Eigen::Vector2d result_state;

  std::unordered_map<std::size_t, std::size_t> im_map;
  std::size_t start_id{ 0 };
  std::size_t image_id{ 0 };
  do
  {
    tm(state, result_state);
    start_id = dgnn.add_point(state);
    image_id = dgnn.add_point(result_state);
    im_map[start_id] = image_id;

  } while (state_space_step(*state, 0.5, dimension, lower_bounds, upper_bounds));

  std::cout << "Total points: " << (im_map.size() * 2) << std::endl;
  try
  {
    dgnn.qhull_to_delaunay();
    // dgnn_im.qhull_to_delaunay();

    std::ofstream ofs_ss((prx::out_path + "delaunay_start_states.txt").c_str(), std::ofstream::trunc);
    std::ofstream ofs_im((prx::out_path + "delaunay_end_states.txt").c_str(), std::ofstream::trunc);
    std::ofstream ofs_edges((prx::out_path + "delaunay_edges.txt").c_str(), std::ofstream::trunc);

    // dgnn.to_file(prx::out_path + "delaunay_start_states.txt");
    // dgnn_im.to_file(prx::out_path + "delaunay_end_states.txt");

    for (auto ss_id : im_map)
    {
      start_id = ss_id.first;
      image_id = ss_id.second;
      ofs_ss << start_id << " " << dgnn[start_id]->point.transpose() << "\n";
      ofs_im << image_id << " " << dgnn[image_id]->point.transpose() << "\n";
    }
    for (auto node : dgnn)
    {
      for (auto neighbor : node->neighbors)
      {
        ofs_edges << node->point.transpose() << " ";
        ofs_edges << dgnn[neighbor]->point.transpose() << " ";
        ofs_edges << "\n";
      }
    }

    std::function<std::unordered_set<std::size_t>(const std::size_t&)> get_neighbors = [&](const std::size_t& id) {
      return dgnn[id]->neighbors;
    };
    std::function<double(const std::size_t&, const std::size_t&)> node_distance =
        [&](const std::size_t& a, const std::size_t& b) { return (dgnn[a]->point - dgnn[b]->point).norm(); };

    // std::size_t v_idx{ 85 };
    std::size_t v_idx{ params["v_query"].as<std::size_t>() };
    std::unordered_map<std::size_t, std::unordered_set<std::size_t>> F;

    std::size_t v_im{ im_map[v_idx] };
    F[v_idx] = { v_im };
    auto N = dgnn[v_idx]->neighbors;
    // std::unordered_set<std::size_t> Y{};
    // for (auto n : N)
    // {
    //   Y.insert(im_map[n]);
    // }
    // PRX_DEBUG_ITERABLE("Y: ", Y);
    // F[v_idx].insert(Y.begin(), Y.end());
    // for (auto y : Y)
    for (auto n : N)
    {
      if (im_map.count(n) > 0)  // if n has an image (pre-computed)
      {
        const std::size_t y{ im_map[n] };
        std::vector<std::size_t> sp = dijkstra_t::shortest_path(v_im, y, get_neighbors, node_distance);
        PRX_DEBUG_VAR_3(n, v_im, y);
        PRX_DEBUG_ITERABLE("sp: ", sp);
        F[v_idx].insert(sp.begin(), sp.end());
      }
      else
      {
        // F[v_idx].insert(n); // <= this produces a unconected set
      }
    }

    PRX_DEBUG_ITERABLE("F: ", F[v_idx]);

    std::ofstream ofs_sites;
    std::ofstream ofs_voronoi;
    std::string sites_filename(prx::out_path + "delaunay_ss_f.txt");
    std::string voronoi_filename(prx::out_path + "delaunay_es_f.txt");
    ofs_sites.open(sites_filename.c_str(), std::ofstream::trunc);
    ofs_voronoi.open(voronoi_filename.c_str(), std::ofstream::trunc);

    ofs_sites << v_idx << " " << dgnn[v_idx]->point.transpose() << "\n";
    for (auto idx : N)
    {
      ofs_sites << idx << " " << dgnn[idx]->point.transpose() << "\n";
    }
    for (auto idx : F[v_idx])
    {
      ofs_voronoi << idx << " " << dgnn[idx]->point.transpose() << "\n";
    }

    // Eigen::Vector2d query_node{ 0.0, 0.0 };
    // auto central_nodes = dgnn.get_gnn()->radius_and_closest_query(query_node, 1.0);

    // std::cout << "Total central nodes: " << central_nodes.size() << "\n";
    // auto delaunay_nodes = dgnn.get_nodes();

    // std::queue<prx::utilities::delaunay_graph_t<2>::NodePtr> to_explore;
    // std::unordered_map<std::size_t, bool> explored_nodes;
    // explored_nodes[delaunay_nodes.begin()->first] = true;
    // to_explore.push(delaunay_nodes.begin()->second);

    // while (!to_explore.empty())
    // {
    //   auto current_node = to_explore.front();
    //   for (auto neighbor : current_node->neighbors)
    //   {
    //     auto site = dgnn[neighbor];
    //     prx_assert(site != current_node, "Current node is its own neighbor!");
    //     ofs_sites << current_node->point.transpose() << " ";
    //     ofs_sites << site->point.transpose() << "\n";
    //     if (explored_nodes.count(neighbor) == 0)
    //     {
    //       to_explore.push(site);
    //       explored_nodes[neighbor] = true;
    //     }
    //   }
    //   to_explore.pop();
    // }

    // This goes through every node, so might print duplicates if the implementation has a bug allowing duplicates
  }
  catch (QhullError& e)
  {
    cerr << e.what() << std::endl;
    return e.errorCode();
  }
  return 0;
}  // main

void printDoubles(const std::vector<double>& doubles)
{
  for (size_t i = 0; i < doubles.size(); i++)
  {
    cout << std::setw(6) << doubles[i] << " ";
  }
}  // printDoubles

void printInts(const std::vector<int>& ints)
{
  for (size_t i = 0; i < ints.size(); i++)
  {
    cout << ints[i] << " ";
  }
}  // printInts

void qconvex_o(const Qhull& qhull)
{
  int dim = qhull.hullDimension();
  int numfacets = qhull.facetList().count();
  int totneighbors = numfacets * dim; /* incorrect for non-simplicial facets, see qh_countfacets */
  cout << dim << "\n" << qhull.points().size() << " " << numfacets << " " << totneighbors / 2 << "\n";
  std::vector<std::vector<double>> points;
  // for(QhullPoint point : qhull.points())
  for (QhullPoints::ConstIterator i = qhull.points().begin(); i != qhull.points().end(); ++i)
  {
    QhullPoint point = *i;
    points.push_back(point.toStdVector());
  }
  // for(std::vector<double> point : points){
  for (size_t j = 0; j < points.size(); ++j)
  {
    std::vector<double> point = points[j];
    size_t n = point.size();
    for (size_t i = 0; i < n; ++i)
    {
      if (i < n - 1)
      {
        cout << std::setw(6) << point[i] << " ";
      }
      else
      {
        cout << std::setw(6) << point[i] << "\n";
      }
    }
  }
  QhullFacetList facets = qhull.facetList();
  std::vector<std::vector<int>> facetVertices;
  // for(QhullFacet f : facets)
  QhullFacetListIterator j(facets);
  while (j.hasNext())
  {
    QhullFacet f = j.next();
    std::vector<int> vertices;
    if (!f.isGood())
    {
      // ignore facet
    }
    else if (!f.isTopOrient() && f.isSimplicial())
    { /* orient the vertices like option 'o' */
      QhullVertexSet vs = f.vertices();
      vertices.push_back(vs[1].point().id());
      vertices.push_back(vs[0].point().id());
      for (int i = 2; i < (int)vs.size(); ++i)
      {
        vertices.push_back(vs[i].point().id());
      }
      facetVertices.push_back(vertices);
    }
    else
    { /* note: for non-simplicial facets, this code does not duplicate option 'o', see qh_facet3vertex and
         qh_printfacetNvertex_nonsimplicial */
      // for(QhullVertex vertex : f.vertices()){
      QhullVertexSetIterator k(f.vertices());
      while (k.hasNext())
      {
        QhullVertex vertex = k.next();
        QhullPoint p = vertex.point();
        vertices.push_back(p.id());
      }
      facetVertices.push_back(vertices);
    }
  }
  // for(std::vector<int> vertices : facetVertices)
  for (size_t k = 0; k < facetVertices.size(); ++k)
  {
    std::vector<int> vertices = facetVertices[k];
    size_t n = vertices.size();
    cout << n << " ";
    for (size_t i = 0; i < n; ++i)
    {
      cout << vertices[i] << " ";
    }
    cout << "\n";
  }
}  // qconvex_o

void qdelaunay_o(const Qhull& qhull, prx::graph_nearest_neighbors_t<Eigen::Vector2d, delaunay_node_t>& gnn,
                 GnnNodes& gnn_nodes)
{
  // The Delaunay diagram is equivalent to the convex hull of a paraboloid, one dimension higher
  int hullDimension = qhull.hullDimension();

  // Input sites as a vector of vectors
  std::vector<std::vector<double>> inputSites;
  QhullPoints points = qhull.points();
  // for(QhullPoint point : points)
  QhullPointsIterator j(points);
  while (j.hasNext())
  {
    QhullPoint point = j.next();
    inputSites.push_back(point.toStdVector());
  }

  // Printer header and Voronoi vertices
  QhullFacetList facets = qhull.facetList();

  // Delaunay regions as a vector of vectors
  std::vector<std::vector<int>> regions;
  // for(QhullFacet f : facets)
  QhullFacetListIterator k(facets);
  while (k.hasNext())
  {
    QhullFacet f = k.next();
    std::vector<int> vertices;
    if (!f.isUpperDelaunay())
    {
      if (!f.isTopOrient() && f.isSimplicial())
      { /* orient the vertices like option 'o' */
        QhullVertexSet vs = f.vertices();
        vertices.push_back(vs[1].point().id());
        vertices.push_back(vs[0].point().id());
        for (int i = 2; i < (int)vs.size(); ++i)
        {
          vertices.push_back(vs[i].point().id());
        }
      }
      else
      { /* note: for non-simplicial facets, this code does not duplicate option 'o', see qh_facet3vertex and
           qh_printfacetNvertex_nonsimplicial */
        // for(QhullVertex vertex : f.vertices()){
        QhullVertexSetIterator i(f.vertices());
        while (i.hasNext())
        {
          QhullVertex vertex = i.next();
          QhullPoint p = vertex.point();
          vertices.push_back(p.id());
        }
      }
      regions.push_back(vertices);
    }
  }
  // for(std::vector<int> vertices : regions)

  for (size_t k2 = 0; k2 < regions.size(); ++k2)
  {
    std::vector<int> vertices = regions[k2];
    size_t n = vertices.size();
    Eigen::RowVector3d centroid{ Eigen::RowVector3d::Zero() };
    gnn_nodes.push_back(std::make_shared<delaunay_node_t>());
    for (size_t i = 0; i < n; ++i)
    {
      // std::vector<double> site = inputSites[vertices[i]];
      Eigen::Map<Eigen::RowVector3d> site{ inputSites[vertices[i]].data(), 3 };
      centroid += site;
      // ofs_sites << site << " ";
      gnn_nodes.back()->sites.emplace_back(site[0], site[1]);
      // gnn_nodes.push_back(std::make_shared<prx::abstract_node_t>());
      // gnn_nodes.back()->set_index(pointCount);
      // gnn_nodes.back()->point = state;
      // for (auto e : site)
      // {
      // centroid ofs_sites << e << " ";
      // }
      // cout << vertices[i] << " ";
    }
    centroid = centroid / n;
    gnn_nodes.back()->point = Eigen::Vector2d(centroid[0], centroid[1]);
    gnn.add_node(gnn_nodes.back().get());
    // ofs_voronoi << centroid << "\n";
    // ofs_sites << "\n";
  }
}  // qdelaunay_o

/***

Sample output for Fi
    rbox y c D2 | qhull v Fi Ta
    [QH9231]10
    [QH9271]5 0 5 [QH9272]0.9933067158065386 [QH9272]-0.115506572685835 [QH9273]-0.3427516509210136 [QH9274]
    ...
    rbox y c D2 | qhull v Fo Ta
    [QH9231]4
    [QH9271]5 3 4 [QH9272]-3.10259629632159e-16 [QH9272]     1 [QH9273]-1.551298148160795e-16 [QH9274]
    ...
*/
void qvoronoi_fifo(Qhull* qhull, const char* printOption)
{
  QhullUser results(qhull->qh());
  qhull->outputQhull(printOption);  // qh_fprintf writes its results into 'results'
  int n = results.numResults();
  if (results.firstCode() != 9231)
  {
    cout << "user_eg3 error (qvoronoi_fifo): 'qhull " << printOption
         << "' did not produce output for 'Fi' or 'Fo'\nqh_fprintf codes: ";
    printInts(results.codes());
    cout << "\n";
    return;
  }
  else if (n != results.numDoubles() || n != results.numInts())
  {
    cout << "user_eg3 error (qvoronoi_fifo): Expecting doubles and ints for " << n << " results.  Got "
         << results.numDoubles() << " doubles and " << results.numInts()
         << " ints.  Did an error occur?\nqh_fprintf codes: ";
    printInts(results.codes());
    cout << "\n";
    return;
  }
  cout << n << "\n";
  for (int i = 0; i < n; ++i)
  {
    printInts(results.intsVector().at(i));
    cout << " ";
    printDoubles(results.doublesVector().at(i));
    cout << "\n";
  }
  if (results.codes().size() < 30)
  {
    cout << "\nMessage codes captured by qh_fprintf in QhullUser.cpp (qhull v Fo Ta):\n";
    printInts(results.codes());
    cout << "\n";
  }
}  // qvoronoi_fifo

void qvoronoi_o(const Qhull& qhull)
{
  int voronoiDimension = qhull.hullDimension() - 1;
  int numfacets = qhull.facetCount();
  size_t numpoints = qhull.points().size();

  // Gather Voronoi vertices
  std::vector<std::vector<double>> voronoiVertices;
  std::vector<double> vertexAtInfinity;
  for (int i = 0; i < voronoiDimension; ++i)
  {
    vertexAtInfinity.push_back(qh_INFINITE);
  }
  voronoiVertices.push_back(vertexAtInfinity);
  // for(QhullFacet facet : qhull.facetList())
  QhullFacetListIterator j(qhull.facetList());
  while (j.hasNext())
  {
    QhullFacet facet = j.next();
    if (facet.visitId() && facet.visitId() < numfacets)
    {
      voronoiVertices.push_back(facet.getCenter().toStdVector());
    }
  }

  // Printer header and Voronoi vertices
  cout << voronoiDimension << "\n" << voronoiVertices.size() << " " << numpoints << " 1\n";
  // for(std::vector<double> voronoiVertex : voronoiVertices)
  for (size_t k = 0; k < voronoiVertices.size(); ++k)
  {
    std::vector<double> voronoiVertex = voronoiVertices[k];
    size_t n = voronoiVertex.size();
    for (size_t i = 0; i < n; ++i)
    {
      cout << voronoiVertex[i] << " ";
    }
    cout << "\n";
  }

  // Gather Voronoi regions
  std::vector<std::vector<int>> voronoiRegions(numpoints);  // qh_printvoronoi calls qh_pointvertex via qh_markvoronoi
  // for(QhullVertex vertex : qhull.vertexList())
  QhullVertexListIterator j2(qhull.vertexList());
  while (j2.hasNext())
  {
    QhullVertex vertex = j2.next();
    size_t numinf = 0;
    std::vector<int> voronoiRegion;
    // for(QhullFacet neighbor : vertex.neighborFacets())
    QhullFacetSetIterator k2(vertex.neighborFacets());
    while (k2.hasNext())
    {
      QhullFacet neighbor = k2.next();
      if (neighbor.visitId() == 0)
      {
        if (!numinf)
        {
          numinf = 1;
          voronoiRegion.push_back(0);  // the voronoiVertex at infinity indicates an unbounded region
        }
      }
      else if (neighbor.visitId() < numfacets)
      {
        voronoiRegion.push_back(neighbor.visitId());
      }
    }
    if (voronoiRegion.size() > numinf)
    {
      int siteId = vertex.point().id();
      if (siteId >= 0 && siteId < int(numpoints))
      {  // otherwise indicate qh.other_points
        voronoiRegions[siteId] = voronoiRegion;
      }
    }
  }

  // Print Voronoi regions by siteId
  // for(std::vector<int> voronoiRegion : voronoiRegions)
  for (size_t k3 = 0; k3 < voronoiRegions.size(); ++k3)
  {
    std::vector<int> voronoiRegion = voronoiRegions[k3];
    size_t n = voronoiRegion.size();
    cout << n;
    for (size_t i = 0; i < n; ++i)
    {
      cout << " " << voronoiRegion[i];
    }
    cout << "\n";
  }
}  // qvoronoi_o

// Nearly the same as qvoronoi_p -- the Voronoi vertex at infinity is not included, hence indices are one less
void qvoronoi_pfn(const Qhull& qhull)
{
  int voronoiDimension = qhull.hullDimension() - 1;
  int numfacets = qhull.facetCount();
  size_t numpoints = qhull.points().size();

  // Gather Voronoi vertices
  std::vector<std::vector<double>> voronoiVertices;
  // for(QhullFacet facet : qhull.facetList())
  QhullFacetListIterator j(qhull.facetList());
  while (j.hasNext())
  {
    QhullFacet facet = j.next();
    if (facet.visitId() && facet.visitId() < numfacets)
    {
      voronoiVertices.push_back(facet.getCenter().toStdVector());
    }
  }

  // Printer header and Voronoi vertices
  cout << voronoiDimension << "\n";
  cout << voronoiVertices.size() << "\n";
  // for(std::vector<double> voronoiVertex : voronoiVertices)
  for (size_t k = 0; k < voronoiVertices.size(); ++k)
  {
    std::vector<double> voronoiVertex = voronoiVertices[k];
    size_t n = voronoiVertex.size();
    for (size_t i = 0; i < n; ++i)
    {
      cout << voronoiVertex[i] << " ";
    }
    cout << "\n";
  }

  // Gather Voronoi regions
  std::vector<std::vector<int>> voronoiRegions(numpoints);  // qh_printvoronoi calls qh_pointvertex via qh_markvoronoi
  // for(QhullVertex vertex : qhull.vertexList()){
  QhullVertexListIterator j2(qhull.vertexList());
  while (j2.hasNext())
  {
    QhullVertex vertex = j2.next();
    size_t numinf = 0;
    std::vector<int> voronoiRegion;
    // for(QhullFacet neighbor : vertex.neighborFacets())
    QhullFacetSetIterator k(vertex.neighborFacets());
    while (k.hasNext())
    {
      QhullFacet neighbor = k.next();
      if (neighbor.visitId() == 0)
      {
        if (!numinf)
        {
          numinf = 1;
          voronoiRegion.push_back(-1);  // -1 indicates the Voronoi vertex at infinity
        }
      }
      else if (neighbor.visitId() < numfacets)
      {
        voronoiRegion.push_back(neighbor.visitId() - 1);
      }
    }
    if (voronoiRegion.size() > numinf)
    {
      int siteId = vertex.point().id();
      if (siteId >= 0 && siteId < int(numpoints))
      {  // otherwise would indicate qh.other_points
        voronoiRegions[siteId] = voronoiRegion;
      }
    }
  }

  // Print Voronoi regions by siteId
  cout << numpoints << "\n";
  // for(std::vector<int> voronoiRegion : voronoiRegions)
  for (size_t j3 = 0; j3 < voronoiRegions.size(); ++j3)
  {
    std::vector<int> voronoiRegion = voronoiRegions[j3];
    size_t n = voronoiRegion.size();
    cout << n;
    for (size_t i = 0; i < n; ++i)
    {
      cout << " " << voronoiRegion[i];
    }
    cout << "\n";
  }
}  // qvoronoi_pfn

int user_eg3(int argc, char** argv)
{
  bool printFacets = false;
  RboxPoints rbox;
  Qhull qhull;
  int readingRbox = 0;
  int readingQhull = 0;
  bool noRboxOutput = false;
  for (int i = 1; i < argc; i++)
  {
    std::cout << "i: " << i << " " << argv[i] << std::endl;
    if (strcmp(argv[i], "eg-100") == 0)
    {
      RboxPoints eg("100");
      Qhull q(eg, "");
      QhullFacetList facets = q.facetList();
      cout << facets;
    }
    else if (strcmp(argv[i], "eg-convex") == 0 && readingQhull > 1)
    {
      cout << "\nInput points and facetlist for '" << qhull.qhullCommand() << "' via C++ classes\n";
      qconvex_o(qhull);
    }
    else if (strcmp(argv[i], "eg-convex") == 0 && !rbox.isEmpty())
    {
      Qhull q(rbox, "");
      cout << "\nInput points and facetlist for convex hull of " << q.rboxCommand() << " via C++ classes\n";
      qconvex_o(q);
      noRboxOutput = true;
    }
    else if (strcmp(argv[i], "eg-convex") == 0)
    {
      cout << "\nA 3-d diamond (rbox d)\n";
      RboxPoints diamond("d");
      cout << diamond;
      cout << "\nInput points and facetlist of its convex hull (qhull o)\n";
      Qhull q(diamond, "o");
      q.setOutputStream(&cout);
      q.outputQhull();
      // q.outputQhull("o") produces the same output
      cout << "\nInput points and facetlist using std::vector and C++ classes\n";
      qconvex_o(q);
      cout << "\nIts outward pointing normals as vector plus offset (qhull n)\n";
      q.outputQhull("n");
    }
    else if (strcmp(argv[i], "eg-delaunay") == 0 && readingQhull > 1 && qhull.isDelaunay())
    {
      cout << "\nVertices and Delaunay regions of paraboloid from '" << qhull.qhullCommand() << "' via C++ classes\n";
      // qdelaunay_o(qhull);
    }
    else if (strcmp(argv[i], "eg-delaunay") == 0 && !rbox.isEmpty())
    {
      cout << "\nDelaunay triangulation of " << rbox.count() << " points as " << rbox.dimension() + 1
           << "-d paraboloid via C++ classes\n";
      PRX_DEBUG_PRINT;
      Qhull q(rbox, "d Qt");
      // qdelaunay_o(q);
      noRboxOutput = true;
    }
    else if (strcmp(argv[i], "eg-delaunay") == 0)
    {
      cout << "\nA 2-d triangle in a square (rbox y c D2)\n";
      RboxPoints triangleSquare("y c D2");
      cout << triangleSquare;
      cout << "\nThe 2-d input sites are lifted to a 3-d paraboloid.\n";
      cout << "A Delaunay region is a facet of the paraboloid's convex hull.\n";
      cout << "\nThe Delaunay triangulation as input sites and Delaunay regions (qhull d o)\n";
      Qhull q(triangleSquare, "d o");
      q.setOutputStream(&cout);
      q.outputQhull();
      // q.outputQhull("o") produces the same output
      cout << "\nThe same results using std::vector and C++ classes\n";
      // qdelaunay_o(q);
    }
    else if (strcmp(argv[i], "eg-voronoi") == 0 && readingQhull > 1 && qhull.isDelaunay())
    {
      cout << "\nVoronoi vertices and regions for '" << qhull.qhullCommand() << "' via C++ classes\n";
      bool isLower;                                         // not used
      int voronoiVertexCount;                               // not used
      qhull.prepareVoronoi(&isLower, &voronoiVertexCount);  // not needed if previous output from Qhull
      qvoronoi_o(qhull);
    }
    else if (strcmp(argv[i], "eg-voronoi") == 0 && !rbox.isEmpty())
    {
      Qhull q(rbox, "v");
      cout << "\nVoronoi vertices and regions for " << q.rboxCommand() << " via C++ classes\n";
      bool isLower;
      int voronoiVertexCount;
      q.prepareVoronoi(&isLower, &voronoiVertexCount);
      qvoronoi_o(q);
      noRboxOutput = true;
    }
    else if (strcmp(argv[i], "eg-voronoi") == 0)
    {
      cout << "\nA 2-d triangle in a square (rbox y c D2)\n";
      RboxPoints triangleSquare("y c D2");
      cout << triangleSquare;
      cout << "\nIts Voronoi diagram as vertices and regions (qhull v o)\n";
      cout << "The Voronoi diagram is the dual of the Delaunay triangulation\n";
      cout << "Voronoi vertices are Delaunay regions, and Voronoi regions are Delaunay input sites\n";
      cout << "The Voronoi vertex at infinity is represented as '-10.101 -10.101'\n";
      Qhull q(triangleSquare, "v o");
      q.setOutputStream(&cout);
      q.outputQhull();
      // q.outputQhull("o") produces the same output
      cout << "\nThe same results using std::vector and C++ classes\n";
      cout << "Qhull::prepareVoronoi assigns facetT.visit_id and vertexT.neighbors\n";
      cout << "The Voronoi regions are rotated by one Voronoi vertex (prepareVoronoi occurs twice)\n";
      bool isLower;
      int voronoiVertexCount;
      q.prepareVoronoi(&isLower, &voronoiVertexCount);  // Also called by q.outputQhull("o"), hence the rotated vertices
      qvoronoi_o(q);
    }
    else if (strcmp(argv[i], "eg-fifo") == 0 && readingQhull > 1 && qhull.isDelaunay())
    {
      cout << "\nVoronoi vertices for '" << qhull.qhullCommand() << "' via C++ classes\n";
      bool isLower;
      int voronoiVertexCount;
      qhull.prepareVoronoi(&isLower, &voronoiVertexCount);
      qvoronoi_pfn(qhull);
      cout << "\nHyperplanes for bounded facets between Voronoi regions via QhullUser and qh_fprintf\n";
      qvoronoi_fifo(&qhull, "Fi");
      cout << "\nHyperplanes for unbounded facets between Voronoi regions via QhullUser and qh_fprintf\n";
      qvoronoi_fifo(&qhull, "Fo");
    }
    else if (strcmp(argv[i], "eg-fifo") == 0 && !rbox.isEmpty())
    {
      Qhull q(rbox, "v");
      cout << "\nVoronoi vertices for " << q.rboxCommand() << " via QhullUser and qh_fprintf\n";
      bool isLower;
      int voronoiVertexCount;
      q.prepareVoronoi(&isLower, &voronoiVertexCount);
      qvoronoi_pfn(q);
      cout << "\nHyperplanes for bounded facets between Voronoi regions via QhullUser and qh_fprintf\n";
      qvoronoi_fifo(&q, "Fi");
      cout << "\nHyperplanes for unbounded facets between Voronoi regions via QhullUser and qh_fprintf\n";
      qvoronoi_fifo(&q, "Fo");
      noRboxOutput = true;
    }
    else if (strcmp(argv[i], "eg-fifo") == 0)
    {
      cout << "\nA 2-d triangle in a square (rbox y c D2)\n";
      RboxPoints triangleSquare("y c D2");
      cout << triangleSquare;
      cout << "\nIts Voronoi vertices (qhull v p)\n";
      cout << "This is the first part of eg-voronoi, but without infinity\n";
      Qhull q(triangleSquare, "v p");
      q.setOutputStream(&cout);
      q.outputQhull();
      cout << "\nIts Voronoi regions (qhull v FN)\n";
      cout << "This is the second part of eg-voronoi, with ids one less\n";
      cout << "Regions are ordered by the corresponding input site.\n";
      q.outputQhull("FN");
      // q.outputQhull("p FN") produces the same outputs
      cout << "\nThe same results as 'qhull v p FN' using std::vector and C++ classes\n";
      cout << "Qhull::prepareVoronoi assigns facet.visit_id and vertex.neighbors\n";
      cout << "prepareVoronoi is also called by q.outputQhull(\"FN\"), hence the rotated vertices\n";
      bool isLower;
      int voronoiVertexCount;
      q.prepareVoronoi(&isLower, &voronoiVertexCount);  // Also called by q.outputQhull("o"), hence the rotated vertices
      qvoronoi_pfn(q);
      cout << "\nHyperplanes for bounded facets between Voronoi regions (qhull v Fi)\n";
      cout << "Each hyperplane is the perpendicular bisector of 2 input sites.\n";
      q.outputQhull("Fi");
      cout << "\nHyperplanes for unbounded rays of unbounded facets (qhull v Fo)\n";
      cout << "Each ray goes through the midpoint of 2 input sites, oriented outwards\n";
      q.outputQhull("Fo");
      cout << "\nThe same result as 'qhull v Fi' using QhullUser and its custom qh_fprintf\n";
      cout << "qh_fprintf captures the output from qh_eachvoronoi in io_r.c (qhull v Fi Fo Ta)\n";
      qvoronoi_fifo(&q, "Fi");
      cout << "\nThe same result as 'qhull v Fo' using QhullUser and its custom qh_fprintf\n";
      qvoronoi_fifo(&q, "Fo");
    }
    else if (strcmp(argv[i], "rbox") == 0)
    {
      if (readingRbox != 0 || readingQhull != 0)
      {
        cerr << "user_eg3 -- \"rbox\" must be first" << endl;
        return 1;
      }
      readingRbox++;
    }
    else if (strcmp(argv[i], "qhull") == 0 || strcmp(argv[i], "qhull-cout") == 0)
    {
      PRX_DEBUG_PRINT;
      if (readingQhull)
      {
        cerr << "user_eg3 -- only one \"qhull\" or \"qhull-cout\" allowed." << endl;
        return 1;
      }
      if (strcmp(argv[i], "qhull-cout") == 0)
      {
        qhull.setOutputStream(&cout);
      }
      if (rbox.isEmpty())
      {
        if (readingRbox)
        {
          if (rbox.dimension() == 0)
          {
            rbox.setDimension(2);
          }
          rbox.appendPoints("10");
        }
        else
        {
          cerr << "Enter dimension followed by count followed by coordinates.  End with ^Z (Windows) or ^D (Unix).\n";
          rbox.appendPoints(cin);
        }
      }
      readingQhull++;
      readingRbox = 0;
    }
    else if (strcmp(argv[i], "facets") == 0)
    {
      printFacets = true;
    }
    else if (readingRbox)
    {
      PRX_DEBUG_PRINT;
      readingRbox++;
      cerr << "rbox " << argv[i] << endl;
      rbox.appendPoints(argv[i]);
      if (rbox.hasRboxMessage())
      {
        cerr << "user_eg3 " << argv[i] << " -- " << rbox.rboxMessage();
        return rbox.rboxStatus();
      }
    }
    else if (readingQhull)
    {
      if (readingQhull == 1)
      {
        qhull.runQhull(rbox, argv[i]);
        qhull.outputQhull();
      }
      else
      {
        qhull.outputQhull(argv[i]);
      }
      readingQhull++;
      if (qhull.hasQhullMessage())
      {
        cerr << "\nResults of " << argv[i] << "\n" << qhull.qhullMessage();
        qhull.clearQhullMessage();
      }
    }
    else
    {
      cerr << "user_eg3 error: Expecting eg-100, eg-convex, eg-delaunay, eg-voronoi, eg-fifo, qhull, qhull-cout, or "
              "rbox.  Got "
           << argv[i] << endl;
      return 1;
    }
  }  // foreach argv
  if (readingRbox && !noRboxOutput)
  {
    PRX_DEBUG_PRINT;
    cout << rbox;
    return 0;
  }
  if (readingQhull == 1)
  {  // e.g., rbox 10 qhull
    PRX_DEBUG_PRINT;
    qhull.runQhull(rbox, "");
    qhull.outputQhull();
    if (qhull.hasQhullMessage())
    {
      cerr << "\nResults of qhull\n" << qhull.qhullMessage();
      qhull.clearQhullMessage();
    }
  }
  if (qhull.hasOutputStream())
  {
    return 0;
  }
  if (printFacets)
  {
    PRX_DEBUG_PRINT;
    QhullFacetList facets = qhull.facetList();
    cout << "\nFacets created by Qhull::runQhull()\n" << facets;
  }
  PRX_DEBUG_PRINT;
  return 0;
}  // user_eg3
