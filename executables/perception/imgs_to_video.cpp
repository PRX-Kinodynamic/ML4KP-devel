#include <algorithm>
#include <fstream>
#include <iostream>
#include <signal.h>
#include <unordered_map>

#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/csv_reader.hpp"
#include "prx/utilities/general/logger.hpp"
#include "prx/utilities/general/type_convertions.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <chrono>
#include <thread>

#include <opencv2/videoio.hpp>

int main(int argc, char** argv)
{
  auto params = prx::param_loader("executables/perception/imgs_to_video.yaml", argc, argv);

  const std::string imgs_path{ params["images_path"].as<>() };
  const std::string video_outname{ params["video_filename"].as<>() };
  const std::string timestamps_file{ params["timestamps_file"].as<>() };
  const std::size_t img_name_column{ params["img_name_column"].as<std::size_t>() };
  const std::size_t timestamp_column{ params["timestamps_column"].as<std::size_t>() };
  const int width{ params["video_width"].as<int>() };
  const int height{ params["video_height"].as<int>() };

  prx::utilities::csv_reader_t reader(timestamps_file, ' ');

  const int fourcc{ cv::VideoWriter::fourcc('m', 'p', '4', 'v') };
  const double fps{ 24 };
  const double rate{ 1.0 / fps };
  // const cv::Size vid_size(height, width);
  PRX_DEBUG_VAR_1(rate);

  double duration{ 0 };

  std::vector<std::size_t> columns_ids = { img_name_column, timestamp_column };
  std::vector<std::vector<std::string>> columns = reader.read_columns<std::string>(columns_ids);
  double prev_time{ prx::utilities::convert_to<double>(columns[1][0]) };

  prx::progress_bar_t bar(columns[0].size() - 1);
  cv::VideoWriter output_video;
  PRX_DEBUG_VAR_1(video_outname);
  PRX_DEBUG_VAR_2(height, width);
  const cv::Size size_out(width, height);
  output_video.open(video_outname, fourcc, fps, size_out, true);
  prx_assert(output_video.isOpened(), "Failed to open video output!");
  cv::Mat frame(height, width, CV_8UC3);
  double ti{ prev_time };
  for (auto i : bar)
  {
    const std::string file_to_read{ imgs_path + columns[0][i] };
    const double timestamp{ prx::utilities::convert_to<double>(columns[1][i + 1]) };

    frame = cv::imread(file_to_read, cv::IMREAD_COLOR);
    cv::resize(frame, frame, size_out, cv::INTER_LINEAR);
    for (; ti < timestamp; ti += rate)
    {
      output_video << frame;
    }
  }
  output_video.release();
  return 0;
}