#include <algorithm>
#include <fstream>
#include <iostream>
#include <signal.h>
#include <unordered_map>

#include "prx/utilities/defs.hpp"
#include "prx/external/aruco_nano.h"
#include "prx/utilities/general/logger.hpp"
#include "prx/utilities/general/zipped_iter.hpp"

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

extern "C" {
#include "apriltag.h"
#include "tag36h11.h"
#include "tag25h9.h"
#include "tag16h5.h"
#include "tagCircle21h7.h"
#include "tagCircle49h12.h"
#include "tagCustom48h12.h"
#include "tagStandard41h12.h"
#include "tagStandard52h13.h"
#include "common/getopt.h"
}

apriltag_family_t* family_tag;
apriltag_detector_t* detector_tag;

using TagSquare = Eigen::Matrix<double, 4, 2>;

void detect_tags(prx::param_loader& params, std::string file_to_read, std::vector<TagSquare>& tags,
                 std::vector<std::size_t>& tags_ids)
{
  cv::VideoCapture cap(file_to_read);
  const int display_wait{ params["display_wait"].as<int>() };
  const bool display{ params["display"].as<bool>() };

  cv::Mat frame, gray;
  cap.read(frame);
  cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

  // Make an image_u8_t header for the Mat data
  image_u8_t im = { .width = gray.cols, .height = gray.rows, .stride = gray.cols, .buf = gray.data };
  zarray_t* detections = apriltag_detector_detect(detector_tag, &im);

  for (int i = 0; i < zarray_size(detections); i++)
  {
    apriltag_detection_t* det;
    zarray_get(detections, i, &det);

    tags.emplace_back(TagSquare::Zero());
    tags.back() << det->p[0][0], det->p[0][1],  // no-lint
        det->p[1][0], det->p[1][1],             // no-lint
        det->p[2][0], det->p[2][1],             // no-lint
        det->p[3][0], det->p[3][1];
    tags_ids.push_back(det->id);
    if (display)
    {
      cv::line(frame, cv::Point(det->p[0][0], det->p[0][1]), cv::Point(det->p[1][0], det->p[1][1]),
               cv::Scalar(0, 0xff, 0), 2);
      cv::line(frame, cv::Point(det->p[0][0], det->p[0][1]), cv::Point(det->p[3][0], det->p[3][1]),
               cv::Scalar(0, 0, 0xff), 2);
      cv::line(frame, cv::Point(det->p[1][0], det->p[1][1]), cv::Point(det->p[2][0], det->p[2][1]),
               cv::Scalar(0xff, 0, 0), 2);
      cv::line(frame, cv::Point(det->p[2][0], det->p[2][1]), cv::Point(det->p[3][0], det->p[3][1]),
               cv::Scalar(0xff, 0, 0), 2);

      std::stringstream ss;
      ss << det->id;
      std::string text = ss.str();
      int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
      double fontscale = 1.0;
      int baseline;
      cv::Size textsize = cv::getTextSize(text, fontface, fontscale, 2, &baseline);
      cv::putText(frame, text, cv::Point(det->c[0] - textsize.width / 2, det->c[1] + textsize.height / 2), fontface,
                  fontscale, cv::Scalar(0xff, 0x99, 0), 2);
    }
  }
  apriltag_detections_destroy(detections);

  if (display)
  {
    cv::imshow("Video", frame);
    cv::waitKey(display_wait);
  }
}

int main(int argc, char* argv[])
{
  auto params = prx::param_loader("executables/tensegrity/tag_processing.yaml", argc, argv);

  family_tag = tagStandard52h13_create();
  detector_tag = apriltag_detector_create();
  apriltag_detector_add_family(detector_tag, family_tag);

  const std::string path = params["imgs_dir"].as<>();

  prx::logger_t tag_outfile(path + params["out_file"].as<>());

  TagSquare tag;
  std::size_t tag_id{ 0 };
  std::vector<TagSquare> tags;
  std::vector<std::size_t> tags_ids;
  const std::regex img_id_regex(params["img_id_regex"].as<>());
  for (const auto& entry : std::filesystem::directory_iterator(path))
  {
    std::cout << entry.path() << std::endl;
    std::cout << entry.path().filename() << std::endl;
    const std::string filename{ entry.path().filename() };
    tags.clear();
    tags_ids.clear();
    detect_tags(params, entry.path(), tags, tags_ids);

    std::smatch m;
    std::regex_search(filename, m, img_id_regex);
    if (m.size() != 1)
    {
      prx_warn("Not a single match to: " << filename << "... Skipping");
      continue;
    }
    std::string img_id{ m[0] };
    for (auto t : prx::zip_iters(tags, tags_ids))
    {
      std::tie(tag, tag_id) = prx::unzip(t);
      tag_outfile.log<false>(img_id, tag_id);
      tag_outfile.log<false>(tag(0, 0), tag(0, 1));
      tag_outfile.log<false>(tag(1, 0), tag(1, 1));
      tag_outfile.log<false>(tag(2, 0), tag(2, 1));
      tag_outfile.log(tag(3, 0), tag(3, 1));
    }
    // return 0;
  }
}