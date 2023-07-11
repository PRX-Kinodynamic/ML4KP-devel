/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SelfCalibrationExample.cpp
 * @brief   Based on VisualSLAMExample, but with unknown (yet fixed) calibration.
 * @author  Frank Dellaert
 */

/*
 * See the detailed documentation in Visual SLAM.
 * The only documentation below with deal with the self-calibration.
 */

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>

// We will also need a camera object to hold calibration information and perform projections.
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3_S2.h>
// Camera observations of landmarks (i.e. pixel coordinates) will be stored as Point2 (x, y).
#include <gtsam/geometry/Point2.h>

// Inference and optimization
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/Values.h>

// SFM-specific factors
#include <gtsam/slam/GeneralSFMFactor.h>  // does calibration !

// Standard headers
#include <vector>

using namespace std;
using namespace gtsam;

std::vector<gtsam::Point3> createPoints()
{
  // Create the set of ground-truth landmarks
  std::vector<gtsam::Point3> points;
  points.push_back(gtsam::Point3(10.0, 10.0, 10.0));
  points.push_back(gtsam::Point3(-10.0, 10.0, 10.0));
  points.push_back(gtsam::Point3(-10.0, -10.0, 10.0));
  points.push_back(gtsam::Point3(10.0, -10.0, 10.0));
  points.push_back(gtsam::Point3(10.0, 10.0, -10.0));
  points.push_back(gtsam::Point3(-10.0, 10.0, -10.0));
  points.push_back(gtsam::Point3(-10.0, -10.0, -10.0));
  points.push_back(gtsam::Point3(10.0, -10.0, -10.0));

  return points;
}

std::vector<gtsam::Point2> get_pixels()
{
  std::vector<gtsam::Point2> points;

  points.push_back(gtsam::Point2(477.074, 636.177));
  points.push_back(gtsam::Point2(463.181, 637.413));
  points.push_back(gtsam::Point2(463.626, 650.826));
  points.push_back(gtsam::Point2(477.313, 649.835));

  points.push_back(gtsam::Point2(821.25, 308.805));
  points.push_back(gtsam::Point2(822.358, 296.231));
  points.push_back(gtsam::Point2(810.894, 296.093));
  points.push_back(gtsam::Point2(809.86, 308.358));

  points.push_back(gtsam::Point2(476.34, 282.849));
  points.push_back(gtsam::Point2(476.121, 268.779));
  points.push_back(gtsam::Point2(462.138, 268.667));
  points.push_back(gtsam::Point2(462.358, 282.193));

  points.push_back(gtsam::Point2(807.947, 605.986));
  points.push_back(gtsam::Point2(808.51, 617.955));
  points.push_back(gtsam::Point2(820.016, 617.151));
  points.push_back(gtsam::Point2(819.615, 604.706));

  return points;
}

std::vector<gtsam::Point3> get_landmarks()
{
  std::vector<gtsam::Point3> points;

  /*  (0,0)                     (1000,0)
   *           p2           p1
   *
   *           p0           p3
   *  (0,1000)                  (1000,1000)
   */

  points.push_back(gtsam::Point3(-15, -18, 1) + gtsam::Point3(0.1, 0.1, 0));
  points.push_back(gtsam::Point3(-15, -18, 1) + gtsam::Point3(0.1, -0.1, 0));
  points.push_back(gtsam::Point3(-15, -18, 1) + gtsam::Point3(-0.1, -0.1, 0));
  points.push_back(gtsam::Point3(-15, -18, 1) + gtsam::Point3(-0.1, 0.1, 0));

  points.push_back(gtsam::Point3(15, 18, 1) + gtsam::Point3(0.1, 0.1, 0));
  points.push_back(gtsam::Point3(15, 18, 1) + gtsam::Point3(0.1, -0.1, 0));
  points.push_back(gtsam::Point3(15, 18, 1) + gtsam::Point3(-0.1, -0.1, 0));
  points.push_back(gtsam::Point3(15, 18, 1) + gtsam::Point3(-0.1, 0.1, 0));

  points.push_back(gtsam::Point3(-15, 18, 1) + gtsam::Point3(0.1, 0.1, 0));
  points.push_back(gtsam::Point3(-15, 18, 1) + gtsam::Point3(0.1, -0.1, 0));
  points.push_back(gtsam::Point3(-15, 18, 1) + gtsam::Point3(-0.1, -0.1, 0));
  points.push_back(gtsam::Point3(-15, 18, 1) + gtsam::Point3(-0.1, 0.1, 0));

  points.push_back(gtsam::Point3(15, -18, 1) + gtsam::Point3(0.1, 0.1, 0));
  points.push_back(gtsam::Point3(15, -18, 1) + gtsam::Point3(0.1, -0.1, 0));
  points.push_back(gtsam::Point3(15, -18, 1) + gtsam::Point3(-0.1, -0.1, 0));
  points.push_back(gtsam::Point3(15, -18, 1) + gtsam::Point3(-0.1, 0.1, 0));

  return points;
}

/* ************************************************************************* */
std::vector<gtsam::Pose3>
createPoses(const gtsam::Pose3& init = gtsam::Pose3(gtsam::Rot3::Ypr(M_PI / 2, 0, -M_PI / 2), gtsam::Point3(30, 0, 0)),
            const gtsam::Pose3& delta = gtsam::Pose3(gtsam::Rot3::Ypr(0, -M_PI / 4, 0),
                                                     gtsam::Point3(sin(M_PI / 4) * 30, 0, 30 * (1 - sin(M_PI / 4)))),
            int steps = 8)
{
  // Create the set of ground-truth poses
  // Default values give a circular trajectory, radius 30 at pi/4 intervals, always facing the circle center
  std::vector<gtsam::Pose3> poses;
  int i = 1;
  poses.push_back(init);
  for (; i < steps; ++i)
  {
    poses.push_back(poses[i - 1].compose(delta));
  }

  return poses;
}

int main(int argc, char* argv[])
{
  // Create the set of ground-truth
  // vector<Point3> points = createPoints();
  // vector<Pose3> poses = createPoses();
  std::vector<Point2> pixels = get_pixels();
  std::vector<Point3> landmarks = get_landmarks();

  // Create the factor graph
  NonlinearFactorGraph graph;

  gtsam::Pose3 p0 = gtsam::Pose3(gtsam::Rot3::Ypr(0, 0, 0), gtsam::Point3(0, 0, 0));
  // Add a prior on pose x1.
  auto poseNoise = noiseModel::Diagonal::Sigmas(
      (Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.3)).finished());  // 30cm std on x,y,z 0.1 rad on
                                                                                  // roll,pitch,yaw
  graph.addPrior(Symbol('x', 0), p0, poseNoise);

  // Simulated measurements from each camera pose, adding them to the factor
  // graph
  auto measurementNoise = noiseModel::Isotropic::Sigma(2, 1.0);
  for (size_t i = 0; i < pixels.size(); ++i)
  {
    // The only real difference with the Visual SLAM example is that here we
    // use a different factor type, that also calculates the Jacobian with
    // respect to calibration
    graph.emplace_shared<GeneralSFMFactor2<gtsam::Cal3_S2> >(pixels[i], measurementNoise, Symbol('x', 0),
                                                             Symbol('l', i), Symbol('K', 0));
  }

  // Add a prior on the position of the first landmark.
  auto pointNoise = noiseModel::Isotropic::Sigma(3, 0.1);
  graph.addPrior(Symbol('l', 0), landmarks[0],
                 pointNoise);  // add directly to graph

  // Add a prior on the calibration.
  auto calNoise = noiseModel::Diagonal::Sigmas((Vector(5) << 500, 500, 0.1, 100, 100).finished());
  // graph.addPrior(Symbol('K', 0), K, calNoise);

  // Create the initial estimate to the solution
  // now including an estimate on the camera calibration parameters
  Values initialEstimate;
  initialEstimate.insert(Symbol('K', 0), Cal3_S2(60.0, 60.0, 0.0, 45.0, 45.0));
  initialEstimate.insert(Symbol('x', 0), p0);
  for (size_t j = 0; j < landmarks.size(); ++j)
  {
    initialEstimate.insert<Point3>(Symbol('l', j), landmarks[j]);
  }

  /* Optimize the graph and print results */
  Values result = DoglegOptimizer(graph, initialEstimate).optimize();
  result.print("Final results:\n");

  return 0;
}
