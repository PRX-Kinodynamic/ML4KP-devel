#include <algorithm>
#include <fstream>
#include <iostream>
#include <signal.h>
#include <unordered_map>

#include "prx/utilities/defs.hpp"
#include "prx/external/aruco_nano.h"
#include "prx/utilities/general/logger.hpp"

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

bool record = false;

// camera parameters
cv::Mat camera_matrix, distCoeffs;
std::unordered_map<std::string, cv::Mat_<double>> camera_parameters{};
std::vector<cv::Vec3d> rvecs;
std::vector<cv::Vec3d> tvecs;

int main(int argc, char** argv)
{
  auto params = prx::param_loader("executables/perception/aruco_from_video.yaml", argc, argv);

  // auto fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');

  cv::VideoCapture cap(params["video_file"].as<>());
  int display_wait{ params["display_wait"].as<int>() };
  // cv::VideoCapture cap(camera_index, cv::CAP_V4L2);

  bool display_video{ params["display_video"].as<bool>() };

  if (!cap.isOpened())
  {
    std::cout << "Error opening the video source!" << std::endl;
    exit(-1);
  }

  cv::Mat frm;
  cv::Mat frm_white, frm_black;

  // cv::Ptr<cv::aruco::Dictionary> dictionary =
  //     cv::makePtr<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100));
  cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);

  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f>> markerCorners;
  std::vector<std::vector<cv::Point2f>> rejected;
  std::vector<double> aux_camera_matrix = {
    602.5044530566524, 0, 317.5438639395286, 0, 749.0014813397526, 238.0332830202925, 0, 0, 1
  };
  std::vector<double> aux_dist_coeff = { -0.588829911578861, 0.1946139646350442, 0.1632790838201396,
                                         0.09344776681021283, 0.6370174824639985 };
  cv::Mat camera_matrix = cv::Mat(aux_camera_matrix, true).reshape(0, 3);
  cv::Mat camera_dist_coeffs = cv::Mat(aux_dist_coeff, true);
  // cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::makePtr<cv::aruco::DetectorParameters>();
  cv::aruco::DetectorParameters parameters = cv::aruco::DetectorParameters();
  parameters.perspectiveRemovePixelPerCell = 1;
  parameters.adaptiveThreshWinSizeStep = 1;
  parameters.adaptiveThreshConstant = 25;
  parameters.minMarkerPerimeterRate = 0.05;
  parameters.aprilTagCriticalRad = 5;
  parameters.aprilTagQuadSigma = 0.5;
  parameters.aprilTagDeglitch = 2;
  parameters.errorCorrectionRate = 1;
  // parameters.aprilTagMaxLineFitMse = 3;
  // parameters.useAruco3Detection = 3;
  // parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_APRILTAG;
  bool finish_recording = false;

  prx::logger_t locs_logger(prx::out_path + "/aruco/" + params["locations_filename"].as<>(), " ");
  prx::logger_t corner_logger(prx::out_path + "/aruco/" + params["corners_filename"].as<>());
  cv::aruco::ArucoDetector detector(dictionary, parameters);
  // std::cout << "cap.read(frm) " << cap.read(frm) << std::endl;
  int marker_id;

  Eigen::Matrix3d kernel_eigen;
  kernel_eigen << 0, -1, 0, -1, 5, -1, 0, -1, 0;
  cv::Mat kernel;
  cv::eigen2cv(kernel_eigen, kernel);
  int total_markers{ 0 };
  int alpha_slider{ 0 };
  while (cap.read(frm))
  {
    // cv::aruco::detectMarkers(frm, dictionary, markerCorners, markerIds, parameters);
    // put estimation code here
    // cv::fastNlMeansDenoisingColored(frm, frm, 10, 10, 7, 21);
    // detector.detectMarkers(frm, markerCorners, markerIds, rejected);
    // cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.155, camera_matrix, camera_dist_coeffs, rvecs, tvecs);
    // cv::aruco::solvePnP(markerCorners, 0.155, camera_matrix, camera_dist_coeffs, rvecs, tvecs);

    // cv::Vec3d r, t;
    // for (int i = 0; i < markerIds.size(); ++i)
    // {
    //   locs_logger.log(markerIds[i], rvecs[i][0], rvecs[i][1], rvecs[i][2], tvecs[i][0], tvecs[i][1], tvecs[i][2]);
    //   // locs_logger.log("\n");
    // }
    // auto start = std::chrono::steady_clock::now();
    // cv::filter2D(frm, frm, -1, kernel);
    cv::cvtColor(frm, frm_black, cv::COLOR_RGB2GRAY);
    cv::threshold(frm_black, frm_black, 210, 250, cv::THRESH_BINARY);
    // cv::threshold(frm, frm_white, 0, 50, cv::THRESH_BINARY_INV);
    // cv::threshold(frm, frm, 0, 50, cv::THRESH_BINARY);
    // cv::GaussianBlur(frm, frm, cv::Size(5, 5), 0);
    // cv::threshold(frm, frm, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);
    // frm = frm_white + frm_black;
    // cv::add(frm_white, frm_black, frm);
    cv::cvtColor(frm_black, frm, cv::COLOR_GRAY2RGB);
    std::vector<aruconano::Marker> markers = aruconano::MarkerDetector::detect(frm);

    // get_location(markerIds, markerCorners);

    if (display_video)
    {
      // std::vector<cv::Point2f> marker;
      // for (auto t : prx::zip_iters(markerIds, markerCorners))
      // {
      //   std::tie(marker_id, marker) = prx::unzip(t);
      //   // std::cout << "Id: " << marker_id << std::endl;
      //   corner_logger.log(marker_id, marker[0].x, marker[0].y, marker[1].x, marker[1].y, marker[2].x, marker[2].y,
      //                     marker[3].x, marker[3].y);
      //   // std::cout << "------" << std::endl;
      // }
      for (auto e : markers)
      {
        total_markers++;
        e.draw(frm);
        // std::cout << e << std::endl;
      }
      // cv::aruco::drawDetectedMarkers(frm, markerCorners, markerIds);
      // cv::aruco::drawDetectedMarkers(frm, rejected, markerIds);
      // cv::aruco::drawDetectedMarkers(frm, rejected, std::vector<int>(rejected.size(), 10));
      cv::imshow("Video", frm);
      cv::waitKey(display_wait);
    }
  }

  std::cout << "Total Markers: " << total_markers << std::endl;
  std::cout << "Finished." << std::endl;
  return 0;
}