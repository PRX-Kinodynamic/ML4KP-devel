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

cv::Ptr<cv::aruco::CharucoBoard> board;

cv::aruco::CharucoParameters charucoParams;
bool read_chessboards(std::string im, std::vector<std::vector<cv::Point2f>>& allCorners,
                      std::vector<std::vector<int>>& allIds, cv::Size& size)
{
  // Charuco base pose estimation.
  // printf("POSE ESTIMATION STARTS:\n");

  // #SUB PIXEL CORNER DETECTION CRITERION

  auto aruco_dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
  // auto board_3x3 = cv::aruco::CharucoBoard(cv::Size(3, 3), 0.115, .09, aruco_dict);
  auto detector = cv::aruco::CharucoDetector(*board);

  cv::Mat gray;
  bool new_detection{ false };
  // for (auto im : images)
  // {
  std::vector<cv::Point2f> corners;
  std::vector<std::vector<cv::Point2f>> marker_corners;
  std::vector<int> ids, marker_ids;
  // std::cout << im << std::endl;
  auto frame = cv::imread(im);
  // cv::imshow("Image", frame);
  // cv::waitKey(100);
  cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
  // corners, ids, markerCorners, markerIds = detector.detectBoard(gray)
  detector.detectBoard(gray, corners, ids, marker_corners, marker_ids);

  // #print(corners)
  if (corners.size() > 3)
  {
    std::cout << "=> Processing image " << im << std::endl;
    cv::aruco::drawDetectedCornersCharuco(frame, corners, ids);
    cv::aruco::drawDetectedMarkers(frame, marker_corners, marker_ids);
    allCorners.push_back(corners);
    allIds.push_back(ids);

    cv::imshow("Image", frame);
    cv::waitKey(100);
    new_detection = true;
  }
  //   # im = PIL.Image.open(frame)
  //   # ax = fig.add_subplot(1,1,1)
  //   # plt.imshow(im)
  //   # ax.axis('off')
  //   # plt.show()

  // #   decimator+=1

  // return allCorners, allIds, imsize
  // }
  size = gray.size();
  return new_detection;
}

void calibrate_camera(std::vector<std::vector<cv::Point2f>>& allCorners, std::vector<std::vector<int>>& allIds,
                      cv::Size& imsize)
{
  //    Calibrates the camera using the dected corners.
  printf("CAMERA CALIBRATION\n");
  auto aruco_dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
  // cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
  // auto board_3x3 = cv::aruco::CharucoBoard(cv::Size(3, 3), 0.115, .09, aruco_dict);
  // cv::Ptr<cv::aruco::CharucoBoard> board_3x3 = new cv::aruco::CharucoBoard(cv::Size(3, 3), 0.098, .076, aruco_dict);
  // cv::Ptr<cv::aruco::CharucoBoard> board_2x2 = new cv::aruco::CharucoBoard(cv::Size(3, 3), 0.098, .076, aruco_dict);
  // cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 1000, 0, imsize.width / 2,  // no-lint
  //                          0, 1000, imsize.height / 2,                           // no-lint
  //                          0, 0, 1);

  // std::vector<double> dist_coeffs = { 1, 1, 1, 1, 1 };
  // flags = (cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_FIX_ASPECT_RATIO)
  double error = cv::aruco::calibrateCameraCharuco(allCorners, allIds, board, imsize, charucoParams.cameraMatrix,
                                                   charucoParams.distCoeffs);
  std::cout << "Error: " << error << std::endl;

  // std::cout << "dist_coeffs: " << dist_coeffs << std::endl;
  // return ret, camera_matrix, distortion_coefficients0, rotation_vectors, translation_vectors
}
int main(int argc, char** argv)
{
  auto params = prx::param_loader("executables/perception/camera_calibration_charuco.yaml", argc, argv);

  bool visualize = true;
  auto aruco_dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
  // auto board_3x3 = cv::aruco::CharucoBoard(cv::Size(3, 3), 0.115, .09, aruco_dict);
  // auto board_2x2 = cv::aruco::CharucoBoard(cv::Size(3, 3), 0.07, .05, aruco_dict);
  board = new cv::aruco::CharucoBoard(cv::Size(3, 3), 0.068, .052, aruco_dict);
  // board = new cv::aruco::CharucoBoard(cv::Size(2, 2), 0.097, .075, aruco_dict);
  // #imboard = aruco.drawPlanarBoard(board_3x3, (2000, 2000), 10, 10, imboard)
  cv::Mat imboard;
  board->generateImage(cv::Size(2000, 2000), imboard);
  // #board_3x3.draw((2000, 2000))
  // #cv2.imwrite(workdir + "chessboard.tiff", imboard)
  if (visualize)
  {
    cv::imshow("Image", imboard);
    cv::waitKey(100);
    // fig = plt.figure() ax = fig.add_subplot(1, 1, 1)
    //                             plt.imshow(imboard, cmap = mpl.cm.gray, interpolation = "nearest") ax.axis("off")
    //                                 plt.show()
  }
  std::string datadir = params["images_dir"].as<>();
  std::vector<std::string> images;

  charucoParams.cameraMatrix = (cv::Mat_<double>(3, 3) << 1000, 0, params["image_width"].as<int>() / 2,  // no-lint
                                0, 1000, params["image_height"].as<int>() / 2,                           // no-lint
                                0, 0, 1);
  charucoParams.distCoeffs = (cv::Mat_<double>(5, 1) << 1, 1, 1, 1, 1);

  std::vector<std::vector<cv::Point2f>> allCorners;
  std::vector<std::vector<int>> allIds;
  cv::Size size;
  for (auto const& dir_entry : std::filesystem::directory_iterator{ datadir })
  {
    if (dir_entry.path().extension() == ".png")
    {
      images.push_back(dir_entry.path());
      auto new_detection = read_chessboards(dir_entry.path(), allCorners, allIds, size);
      if (new_detection)
      {
        calibrate_camera(allCorners, allIds, size);
      }
    }
  }

  std::cout << "camera_matrix: " << charucoParams.cameraMatrix << std::endl;
  std::cout << "dist_coeffs: " << charucoParams.distCoeffs << std::endl;
  return 0;
}