#include <algorithm>
#include <fstream>
#include <iostream>
#include <signal.h>
#include <unordered_map>

#include "prx/utilities/defs.hpp"
#include "prx/external/aruco_nano.h"
#include "prx/utilities/general/logger.hpp"
#include "prx/utilities/general/csv_reader.hpp"
#include "prx/factor_graphs/utilities/perception/camera.hpp"
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <chrono>
#include <thread>

#include <opencv2/videoio.hpp>

using namespace prx;
using prx::utilities::csv_reader_t;

void populate_marker(std::vector<std::string>& line, aruconano::Marker& marker, Eigen::Vector3d& midpt)
{
  midpt << 0, 0, 1;
  for (int i = 0; i < 4; ++i)
  {
    const double u{ std::stod(line[8 + i * 2]) };
    const double v{ std::stod(line[8 + i * 2 + 1]) };
    marker[i].x = u;
    marker[i].y = v;
    midpt[0] += u;
    midpt[1] += v;
  }
  midpt[0] = midpt[0] / 4;
  midpt[1] = midpt[1] / 4;
}

int main(int argc, char** argv)
{
  auto params = prx::param_loader("executables/perception/camera_to_world.yaml", argc, argv);
  std::string markers_file = params["markers_file"].as<>();
  csv_reader_t reader(markers_file, ' ');

  Eigen::Matrix3d camera_matrix;
  Eigen::Vector<double, 5> distortion_coeff;
  double marker_length = params["marker_length"].as<double>();
  init_from_container(camera_matrix, params["camera_matrix"].as<std::vector<double>>());
  init_from_container(distortion_coeff, params["distortion_coeff"].as<std::vector<double>>());

  Eigen::Matrix3d rotation;
  Eigen::Vector3d translation;
  Eigen::Matrix<double, 3, 4> perspective_projection{ Eigen::Matrix<double, 3, 4>::Identity() };
  // std::cout << "perspective_projection: " << perspective_projection << std::endl;

  aruconano::Marker marker;
  marker.emplace_back(0, 0);
  marker.emplace_back(0, 0);
  marker.emplace_back(0, 0);
  marker.emplace_back(0, 0);

  Eigen::Transform<double, 3, Eigen::Isometry> transform, T0;
  Eigen::Vector3d midpt;
  Eigen::Vector3d position{ Eigen::Vector3d::Ones() };
  Eigen::Vector3d P0{ Eigen::Vector3d::Ones() };
  Eigen::Vector4d Pos4{ Eigen::Vector4d::Ones() };
  bool first_found{ true };

  std::vector<cv::Point3d> corners = { { -marker_length / 2.f, marker_length / 2.f, 0.f },
                                       { marker_length / 2.f, marker_length / 2.f, 0.f },
                                       { marker_length / 2.f, -marker_length / 2.f, 0.f },
                                       { -marker_length / 2.f, -marker_length / 2.f, 0.f } };
  std::unordered_map<std::string, std::pair<cv::Mat, cv::Mat>> rt_map;
  cv::Mat r_vec, t_vec, Rmat;
  Eigen::Matrix3d rmat_eigen;
  Eigen::Vector3d tvec_eigen;
  bool use_rt_vecs{ false };
  while (reader.has_next_line())
  {
    use_rt_vecs = false;

    // auto line = reader.next_line();
    auto line = reader.next_line("/camera_logitech/pracsys/markers", 0);
    // PRX_DEBUG_ITERABLE(line);
    if (line.size() == 0)
      continue;

    populate_marker(line, marker, midpt);

    if (rt_map.count(line[7]) > 0)
    {
      std::tie(r_vec, t_vec) = rt_map[line[7]];
      use_rt_vecs = true;
    }
    cv::Mat cam_mat, dist_vec;
    cv::eigen2cv(camera_matrix, cam_mat);
    cv::eigen2cv(distortion_coeff, dist_vec);
    cv::solvePnP(corners, marker, cam_mat, dist_vec, r_vec, t_vec, use_rt_vecs, cv::SOLVEPNP_IPPE_SQUARE);
    rt_map[line[7]] = std::make_pair(r_vec, t_vec);
    // std::tie(rotation, translation) = marker.estimatePoseEigenRodrigues(camera_matrix, distortion_coeff,
    // marker_length);
    cv::Rodrigues(r_vec, Rmat);
    cv::cv2eigen(Rmat, rotation);
    cv::cv2eigen(t_vec, translation);
    transform.translation() = translation;
    transform.linear() = rotation;
    // auto aux =
    // Pos4 << midpt, 1;
    // transform *= Eigen::AngleAxisd(PRX_PI, Eigen::Vector3d::UnitY());
    // position.block<2, 1>(0, 0) =
    //     camera_matrix.block<2, 2>(0, 0) * (midpt.block<2, 1>(0, 0) - camera_matrix.block<2, 1>(0, 2));

    // position = camera_matrix.inverse() * midpt;
    // std::cout << line[7] << " " << (translation).transpose() << rotation.eulerAngles(0, 1, 2) << std::endl;
    position = camera_matrix.inverse() * midpt * 3.5;

    position[0] *= -1;
    position[0] *= 0.35;
    if (line[7] == "52" && first_found)
    {
      T0 = transform;
      P0 = position;
      first_found = false;
    }
    else if (first_found)
    {
      continue;
    }
    // position *= 2.75409;
    // auto w = rotation.inverse() * (position - translation);
    // auto w = transform.inverse() * position;
    // transforms[line[7]] = w;
    // auto m = camera_matrix * perspective_projection * transform;
    // auto w = m.inverse() * midpt;
    std::cout << line[7] << " " << (P0 - position).transpose() << " "
              << std::atan2(P0[1] - position[1], P0[0] - position[0]) << std::endl;
    // std::cout << line[7] << " " << (transform.inverse() * position).transpose() << std::endl;
    // std::cout << line[7] << " " << (w * p_i).transpose() << std::endl;
    // std::cout << line[7] << " " << (transform.inverse() * position - P0).transpose() << std::endl;
    // transform.translation() -= T0.translation();
    // std::cout << line[7] << " " << (transform).translation().transpose() << std::endl;
    // std::cout << line[7] << " " << translation.transpose() << std::endl;
    // std::cout << "inv: " << transform.matrix() << std::endl;
    // std::cout << "inv: " << transform.matrix() << std::endl;
    // std::pair<Eigen::Vector3d, Eigen::Vector3d> estimatePoseEigen(Eigen::Matrix3d cameraMatrix,
    //                                                               DistortionVector distCoeffs, double markerSize)
    //                                                               const
  }
}