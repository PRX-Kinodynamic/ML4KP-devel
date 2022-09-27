#define BOOST_AUTO_TEST_MAIN qhull
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <boost/test/unit_test.hpp>
#include "prx/utilities/general/random.hpp"

#include "prx/external/qhull/src/libqhull_r/user_r.h" /* QHULL_CRTDBG */
#include "prx/external/qhull/src/libqhull_r/qset_r.h"
#include "prx/external/qhull/src/libqhull_r/mem_r.h"
#include "prx/external/qhull/src/libqhull_r/geom_r.h"
#include "prx/external/qhull/src/libqhull_r/libqhull_r.h"
#include "prx/external/qhull/src/libqhullcpp/RboxPoints.h"
#include "prx/external/qhull/src/libqhullcpp/QhullError.h"
#include "prx/external/qhull/src/libqhullcpp/QhullQh.h"
#include "prx/external/qhull/src/libqhullcpp/QhullFacet.h"
#include "prx/external/qhull/src/libqhullcpp/QhullFacetList.h"
#include "prx/external/qhull/src/libqhullcpp/QhullFacetSet.h"
#include "prx/external/qhull/src/libqhullcpp/QhullLinkedList.h"
#include "prx/external/qhull/src/libqhullcpp/QhullPoint.h"
#include "prx/external/qhull/src/libqhullcpp/QhullUser.h"
#include "prx/external/qhull/src/libqhullcpp/QhullVertex.h"
#include "prx/external/qhull/src/libqhullcpp/QhullVertexSet.h"
#include "prx/external/qhull/src/libqhullcpp/Qhull.h"

// clang-format off
std::function<double(const double)> point_generator = [](const double& x)
{
	return prx::uniform_random();
};
// clang-format on

BOOST_AUTO_TEST_CASE(qhull)
{
  orgQhull::Qhull qhull;
  const int dim{ 2 };

  std::vector<double> pts{};
  for (int i = 0; i < 100; ++i)
  {
    pts.push_back(point_generator(i));
    pts.push_back(point_generator(i));
  }
  std::cout << "Computing Qhull" << std::endl;
  qhull.runQhull("test", dim, pts.size() / 2, pts.data(), "");
  std::cout << "Done computing Qhull" << std::endl;

  std::ofstream ofs_qhull, ofs_points;
  ofs_qhull.open("qhull_hull.txt", std::ofstream::trunc);
  ofs_points.open("qhull_points.txt", std::ofstream::trunc);

  for (auto pt : qhull.vertexList())
  {
    // ofs_qhull << "i" << std::endl;
    const double x{ pt.point().coordinates()[0] };
    const double y{ pt.point().coordinates()[1] };
    // std::cout << x << " " << y << "\n";
    ofs_qhull << x << " " << y << "\n";
    // ofs_qhull << pt << std::endl;
  }
  // orgQhull::QhullVertex vertex{ qhull.beginVertex() };
  // while (vertex.hasNext())
  // {
  // ofs_qhull << "i" << std::endl;  //<< pt;
  // }
  for (int i = 0; i < pts.size();)
  {
    ofs_points << pts[i] << " ";
    i++;
    ofs_points << pts[i] << "\n";
    i++;
  }

  // for (unsigned i = 0; i < num_states; ++i)
  // {
  //   ofs_map << states[i] << "\n";
  // }
  // ofs_map << "\n";

  // ofs_map.close();
}