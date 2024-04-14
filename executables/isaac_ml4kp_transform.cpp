#include <fstream>
#include <iostream>

#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/rrt_star.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/planners/planner.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"

using Transform = Eigen::Transform<double, 3, Eigen::Isometry>;

using prx::split;
using prx::utilities::convert_to;
int main(int argc, char* argv[])
{
  std::string input;
  Eigen::Vector3d pos;
  Eigen::Quaterniond quat;
  while (getline(std::cin, input))
  {
    std::vector<double> line{ split<double>(input) };
    const double x{ convert_to<double>(line[0]) };
    const double y{ convert_to<double>(line[1]) };
    const double z{ convert_to<double>(line[2]) };

    const double qw{ convert_to<double>(line[3]) };
    const double qx{ convert_to<double>(line[4]) };
    const double qy{ convert_to<double>(line[5]) };
    const double qz{ convert_to<double>(line[6]) };
    quat = Eigen::Quaterniond{ qw, qx, qy, qz };
    pos = Eigen::Vector3d(x, y, z);
  }

  Transform ml4kpWorld_w{ Transform::Identity() };
  Transform pegbottom_w{ Transform::Identity() };
  Transform pegBottom_ml4kp{ Transform::Identity() };
  Transform pegCenter_pegBottom{ Transform::Identity() };
  Transform pegcenter_ml4kp{ Transform::Identity() };

  ml4kpWorld_w.translation() = Eigen::Vector3d(130.0, 0.0, 1040.0);
  // ml4kpWorld_w.translation() = Eigen::Vector3d(0.13, 0, 1.340);
  pegbottom_w.translation() = pos;
  pegbottom_w.linear() = quat.toRotationMatrix();
  pegCenter_pegBottom.translation() = Eigen::Vector3d(0, 0, 25);

  Transform pegCenter_w{ pegbottom_w * pegCenter_pegBottom };
  //pegcenter_ml4kp = ml4kpWorld_w.inverse() * pegCenter_w;
  pegcenter_ml4kp = pegCenter_w * ml4kpWorld_w.inverse();
  pegBottom_ml4kp = pegbottom_w * ml4kpWorld_w.inverse();

  const Eigen::Vector3d out_pos{ pegcenter_ml4kp.translation() };
  const Eigen::Quaterniond out_quat{ pegcenter_ml4kp.rotation() };
  
  PRX_DEBUG_VAR_1(pegCenter_w.translation().transpose());
  PRX_DEBUG_VAR_1(pegcenter_ml4kp.translation().transpose());
  PRX_DEBUG_VAR_1(pegBottom_ml4kp.translation().transpose());
  PRX_DEBUG_VAR_1(out_pos.transpose());
  PRX_DEBUG_VAR_1(out_quat);


  return 0;
}
