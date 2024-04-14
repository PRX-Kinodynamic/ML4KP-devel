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

  std::string input_socket;
  Eigen::Vector3d pos_socket;
  Eigen::Quaterniond quat_socket;

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
    
    const double x_s{ convert_to<double>(line[7]) };
    const double y_s{ convert_to<double>(line[8]) };
    const double z_s{ convert_to<double>(line[9]) };

    const double qw_s{ convert_to<double>(line[10]) };
    const double qx_s{ convert_to<double>(line[11]) };
    const double qy_s{ convert_to<double>(line[12]) };
    const double qz_s{ convert_to<double>(line[13]) };

    quat = Eigen::Quaterniond{ qw, qx, qy, qz };
    pos = Eigen::Vector3d(x, y, z);

    quat_socket = Eigen::Quaterniond{ qw_s, qx_s, qy_s, qz_s };
    pos_socket = Eigen::Vector3d(x_s, y_s, z_s);

  }

  Transform ml4kpWorld_w{ Transform::Identity() };
  
  Transform pegBottom_w{ Transform::Identity() };
  Transform pegCenter_w{ Transform::Identity() };
  
  Transform pegBottom_ml4kp{ Transform::Identity() };
  Transform pegCenter_ml4kp{ Transform::Identity() };

  Transform pegCenter_pegBottom{ Transform::Identity() };


  pegCenter_pegBottom.translation() = Eigen::Vector3d(0, 0, 25);

  ml4kpWorld_w.translation() = pos_socket;
  ml4kpWorld_w.linear() = quat_socket.toRotationMatrix(); 
  //ml4kpWorld_w.translation() = Eigen::Vector3d(130.0, 0.0, 1040.0);
  
  pegCenter_ml4kp.translation() = pos;
  pegCenter_ml4kp.linear() = quat.toRotationMatrix();


  pegBottom_ml4kp = pegCenter_ml4kp * pegCenter_pegBottom.inverse();
  pegBottom_w = ml4kpWorld_w * pegBottom_ml4kp;
  pegCenter_w = pegBottom_w * pegCenter_pegBottom;

  //pegCenter_w = ml4kpWorld_w * pegCenter_ml4kp;
  //pegBottom_w = pegCenter_w * pegCenter_pegBottom.inverse();

  const Eigen::Vector3d out_pos{ pegBottom_w.translation() };
  const Eigen::Quaterniond out_quat{ pegBottom_w.rotation() };

  PRX_DEBUG_VAR_1(pegBottom_w.translation().transpose());
  PRX_DEBUG_VAR_1(pegCenter_w.translation().transpose());
  PRX_DEBUG_VAR_1(pegBottom_ml4kp.translation().transpose());
  PRX_DEBUG_VAR_1(pegCenter_ml4kp.translation().transpose());

  PRX_DEBUG_VAR_1(out_pos.transpose());
  PRX_DEBUG_VAR_1(out_quat);

  return 0;
}
