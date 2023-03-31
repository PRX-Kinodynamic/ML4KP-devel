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
  auto params = prx::param_loader("executables/perception/multi_aruco_fg.yaml", argc, argv);

  // auto fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');

  const std::vector<std::string> sources{ params["video_sources"].as<std::vector<std::string>>() };
  const std::size_t total_sources{ sources.size() };

  std::vector<cv::VideoCapture> video_captures;
  int display_wait{ params["display_wait"].as<int>() };

  std::vector<cv::Mat> frames;
  std::vector<cv::Mat> frames_aux;
  std::for_each(sources.begin(), sources.end(),
                [&](const std::string s) {  // no-lint
                  video_captures.emplace_back(s);
                  if (!video_captures.back().isOpened())
                  {
                    std::cout << "Error opening the video source: " << s << std::endl;
                    exit(-1);
                  }
                  frames.emplace_back(cv::Mat::zeros(cv::Size(512, 700), CV_8UC3));
                  frames_aux.emplace_back(cv::Mat::zeros(cv::Size(512, 700), CV_8UC1));
                });

  bool display_video{ params["display_video"].as<bool>() };

  // cv::Ptr<cv::aruco::Dictionary> dictionary =
  //     cv::makePtr<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100));

  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f>> markerCorners;
  std::vector<std::vector<cv::Point2f>> rejected;

  bool finish_recording = false;

  prx::logger_t locs_logger(prx::out_path + "/aruco/" + params["locations_filename"].as<>(), " ");
  prx::logger_t corner_logger(prx::out_path + "/aruco/" + params["corners_filename"].as<>());
  // std::cout << "cap.read(frm) " << cap.read(frm) << std::endl;
  int marker_id;

  int total_markers{ 0 };
  cv::VideoCapture capture_i;
  cv::Mat frame_i, frame_aux_i;
  std::vector<aruconano::Marker> frame_markers_i;

  cv::Mat frame_to_display;
  std::vector<cv::Mat> frames_found;
  std::vector<std::vector<aruconano::Marker>> markers(total_sources);  // =

  bool available_frames = true;
  while (available_frames)
  {
    available_frames = false;
    frames_found.clear();
    for (auto t : prx::zip_iters(video_captures, frames, frames_aux, markers))
    {
      std::tie(capture_i, frame_i, frame_aux_i, frame_markers_i) = prx::unzip(t);

      if (capture_i.read(frame_i))
      {
        available_frames = true;
        cv::cvtColor(frame_i, frame_aux_i, cv::COLOR_RGB2GRAY);
        cv::threshold(frame_aux_i, frame_aux_i, 210, 250, cv::THRESH_BINARY);
        frame_markers_i = aruconano::MarkerDetector::detect(frame_aux_i);
        // if (display_video)
        // {
        //   cv::cvtColor(frame_aux_i, frame_i, cv::COLOR_GRAY2RGB);
        // }
        for (auto e : frame_markers_i)
        {
          total_markers++;
          e.draw(frame_i);
        }
        frames_found.emplace_back();
        cv::resize(frame_i, frames_found.back(), cv::Size(512, 512), cv::INTER_LINEAR);
      }
      else
      {
        frame_markers_i.clear();
      }
    }

    if (display_video && available_frames)
    {
      // for (auto t : prx::zip_iters(frames, frames_aux, markers))
      // {
      //   std::tie(frame_i, frame_aux_i, frame_markers_i) = prx::unzip(t);
      //   cv::cvtColor(frame_aux_i, frame_i, cv::COLOR_GRAY2RGB);
      // }
      cv::hconcat(frames_found, frame_to_display);
      cv::imshow("Video", frame_to_display);
      cv::waitKey(display_wait);
    }
  }

  std::cout << "Total Markers: " << total_markers << std::endl;
  std::cout << "Finished." << std::endl;
  return 0;
}