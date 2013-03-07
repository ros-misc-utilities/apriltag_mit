/**
 * @file april_tags.cpp
 * @brief Example application for April tags library
 * @author: Michael Kaess
 *
 */

#include <iostream>
#include <cstring>

const std::string usage = "\n"
    "Usage:\n"
    "  apriltags_demo [OPTION...] [deviceID]\n"
    "\n"
    "Options:\n"
    "  -h  -?       show help options\n"
    "  -d           disable graphics\n"
    "  -W           image width (availability depends on camera)\n"
    "  -H           image height (availability depends on camera)\n"
    "\n";

const std::string intro = "\n"
    "April tags test code\n"
    "(C) 2012-2013 Massachusetts Institute of Technology\n"
    "Michael Kaess\n"
    "\n";


#include <sys/time.h>

#include "opencv2/opencv.hpp"

#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag36h11.h"

// for getopt / command line options processing
#include <unistd.h>
extern int optind;
extern char *optarg;


using namespace std;


const char* window_name = "apriltags_demo";


// draw April tag detection on actual image
void draw_detection(cv::Mat& image, const AprilTags::TagDetection& detection) {
  // use corner points detected by line intersection
  std::pair<float, float> p1 = detection.p[0];
  std::pair<float, float> p2 = detection.p[1];
  std::pair<float, float> p3 = detection.p[2];
  std::pair<float, float> p4 = detection.p[3];

  // plot outline
  cv::line(image, cv::Point2f(p1.first, p1.second), cv::Point2f(p2.first, p2.second), cv::Scalar(255,0,0,0) );
  cv::line(image, cv::Point2f(p2.first, p2.second), cv::Point2f(p3.first, p3.second), cv::Scalar(0,255,0,0) );
  cv::line(image, cv::Point2f(p3.first, p3.second), cv::Point2f(p4.first, p4.second), cv::Scalar(0,0,255,0) );
  cv::line(image, cv::Point2f(p4.first, p4.second), cv::Point2f(p1.first, p1.second), cv::Scalar(255,0,255,0) );

  // mark center
  cv::circle(image, cv::Point2f(detection.cxy.first, detection.cxy.second), 8, cv::Scalar(0,0,255,0), 2);

  // print ID
  std::ostringstream strSt;
  strSt << "#" << detection.id;
  cv::putText(image, strSt.str(),
              cv::Point2f(detection.cxy.first + 10, detection.cxy.second + 10),
              cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,255));
}

double tic() {
  struct timeval t;
  gettimeofday(&t, NULL);
  return ((double)t.tv_sec + ((double)t.tv_usec)/1000000.);
}

int main(int argc, char* argv[]) {
  bool draw = true;
  int width = 640;
  int height = 480;

  int c;
  while ((c = getopt(argc, argv, ":h?dW:H:")) != -1) {
    // Each option character has to be in the string in getopt();
    // the first colon changes the error character from '?' to ':';
    // a colon after an option means that there is an extra
    // parameter to this option; 'W' is a reserved character
    switch (c) {
    case 'h':
    case '?':
      cout << intro;
      cout << usage;
      exit(0);
      break;
    case 'd':
      draw = false;
      break;
    case 'W':
      width = atoi(optarg);
      break;
    case 'H':
      height = atoi(optarg);
      break;
    case ':': // unknown option, from getopt
      cout << intro;
      cout << usage;
      exit(1);
      break;
    }
  }

  int device_id = 0;
  if (argc == optind + 1) {
    device_id = atoi(argv[optind]);
  }

  // find any available camera (laptop camera, web cam etc)
  cv::VideoCapture cap(device_id);
  if(!cap.isOpened()) {
    cerr << "ERROR: Can't find video device " << device_id << "\n";
    return -1;
  }
  cap.set(CV_CAP_PROP_FRAME_WIDTH, width);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT, height);
  cout << "Camera successfully opened (ignore error messages above...)" << endl;
  cout << "Actual resolution: "
       << cap.get(CV_CAP_PROP_FRAME_WIDTH) << "x"
       << cap.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;

  // determines which family of April tags is detected
  AprilTags::TagDetector tag_detector(AprilTags::tagCodes36h11);

  if (draw) {
    cv::namedWindow(window_name, 1);
  }
  cv::Mat color;
  cv::Mat gray;

  int frame = 0;
  double last_t = tic();
  while (true) {

    // capture frame
    //    if (frame==0)
    cap >> color;

    cv::cvtColor(color, gray, CV_BGR2GRAY);

    std::vector<AprilTags::TagDetection> detections = tag_detector.extractTags(gray);

    // print out detections
    cout << detections.size() << " tags detected:" << endl;
    for (int i=0; i<detections.size(); i++) {

      cout << "  Id: " << detections[i].id
           << " (Hamming: " << detections[i].hammingDistance << ")";

      if (draw) {
        // also highlight in the image
        draw_detection(color, detections[i]);
      }

      // recovering the relative pose of a tag:

      // NOTE: for this to be accurate, it is necessary to use the
      // actual camera parameters here as well as the actual tag size

      const double tag_size = 0.166; // real side length in meters of square black frame
      const double fx = 600; // camera focal length
      const double fy = 600;
      const double px = gray.cols/2; // camera principal point
      const double py = gray.rows/2;
      Eigen::Matrix4d T =
        detections[i].getRelativeTransform(tag_size, fx, fy, px, py);

      // recovering rotation and translation from the T matrix;
      // converting from camera frame (z forward, x right, y down) to
      // object frame (x forward, y left, z up)
      Eigen::Matrix4d M;
      M <<
        0,  0, 1, 0,
        -1, 0, 0, 0,
        0, -1, 0, 0,
        0,  0, 0, 1;
      Eigen::Matrix4d MT = M*T;
      // translation vector from camera to the April tag
      Eigen::Vector3d trans = MT.col(3).head(3);
      // orientation of April tag with respect to camera
      Eigen::Matrix3d rot = MT.block(0,0,3,3);

      cout << "  distance=" << trans.norm()
           << "m, x=" << trans(0) << ", y=" << trans(1) << ", z=" << trans(2);
      cout << endl;

      // note that for SLAM application it is better to use
      // reprojection error of corner points, as the noise in this
      // relative pose is very non-Gaussian; see iSAM source code for
      // suitable factors

    }

    if (draw) {
      imshow(window_name, color);
    }

    frame++;
    if (frame % 10 == 0) {
      double t = tic();
      cout << "  " << 10./(t-last_t) << " fps" << endl;
      last_t = t;
    }

    // exit if any key pressed
    if (cv::waitKey(1) >= 0) break;
  }

  return 0;
}
