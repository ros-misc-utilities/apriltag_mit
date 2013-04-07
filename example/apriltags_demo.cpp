/**
 * @file april_tags.cpp
 * @brief Example application for April tags library
 * @author: Michael Kaess
 *
 * Opens the first available camera (typically a built in camera in a
 * laptop) and continuously detects April tags in the incoming
 * images. Detections are both visualized in the live image and shown
 * in the text console. Optionally allows selecting of a specific
 * camera in case multiple ones are present and specifying image
 * resolution as long as supported by the camera. Also includes the
 * option to send tag detections via a serial port, for example when
 * running on a Raspberry Pi that is connected to an Arduino.
 */

using namespace std;

#include <iostream>
#include <cstring>
#include <vector>
#include <sys/time.h>

const string usage = "\n"
  "Usage:\n"
  "  apriltags_demo [OPTION...] [deviceID]\n"
  "\n"
  "Options:\n"
  "  -h  -?       show help options\n"
  "  -a           Arduino (send tag ids over serial port)\n"
  "  -d           disable graphics\n"
  "  -C <bbxhh>   Tag family (default 36h11)\n"
  "  -F <fx>      Focal length in pixels\n"
  "  -W <width>   image width (default 640, availability depends on camera)\n"
  "  -H <height>  image height (default 480, availability depends on camera)\n"
  "  -S <size>    Tag size (square black frame) in meters\n"
  "\n";

const string intro = "\n"
    "April tags test code\n"
    "(C) 2012-2013 Massachusetts Institute of Technology\n"
    "Michael Kaess\n"
    "\n";


// OpenCV library for easy access to USB camera and drawing of images
// on screen
#include "opencv2/opencv.hpp"

// April tags detector and various families
#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag16h5.h"
#include "AprilTags/Tag25h7.h"
#include "AprilTags/Tag25h9.h"
#include "AprilTags/Tag36h9.h"
#include "AprilTags/Tag36h11.h"


// Needed for getopt / command line options processing
#include <unistd.h>
extern int optind;
extern char *optarg;

// For Arduino: locally defined serial port access class
#include "Serial.h"


const char* window_name = "apriltags_demo";


// draw one April tag detection on actual image
void draw_detection(cv::Mat& image, const AprilTags::TagDetection& detection) {
  // use corner points detected by line intersection
  pair<float, float> p1 = detection.p[0];
  pair<float, float> p2 = detection.p[1];
  pair<float, float> p3 = detection.p[2];
  pair<float, float> p4 = detection.p[3];

  // plot outline
  cv::line(image, cv::Point2f(p1.first, p1.second), cv::Point2f(p2.first, p2.second), cv::Scalar(255,0,0,0) );
  cv::line(image, cv::Point2f(p2.first, p2.second), cv::Point2f(p3.first, p3.second), cv::Scalar(0,255,0,0) );
  cv::line(image, cv::Point2f(p3.first, p3.second), cv::Point2f(p4.first, p4.second), cv::Scalar(0,0,255,0) );
  cv::line(image, cv::Point2f(p4.first, p4.second), cv::Point2f(p1.first, p1.second), cv::Scalar(255,0,255,0) );

  // mark center
  cv::circle(image, cv::Point2f(detection.cxy.first, detection.cxy.second), 8, cv::Scalar(0,0,255,0), 2);

  // print ID
  ostringstream strSt;
  strSt << "#" << detection.id;
  cv::putText(image, strSt.str(),
              cv::Point2f(detection.cxy.first + 10, detection.cxy.second + 10),
              cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,255));
}

// utility function to provide current system time (used below in
// determining frame rate at which images are being processed)
double tic() {
  struct timeval t;
  gettimeofday(&t, NULL);
  return ((double)t.tv_sec + ((double)t.tv_usec)/1000000.);
}


class Demo {

  AprilTags::TagDetector* m_tagDetector;
  AprilTags::TagCodes m_tagCodes;

  bool m_draw; // draw image and April tag detections?
  bool m_arduino; // send tag detections to serial port?

  int m_width; // image size in pixels
  int m_height;
  double m_tagSize; // April tag side length in meters of square black frame
  double m_fx; // camera focal length in pixels
  double m_fy;
  double m_px; // camera principal point
  double m_py;

  int m_deviceId; // camera id (in case of multiple cameras)
  cv::VideoCapture m_cap;

  Serial m_serial;

public:

  // default constructor
  Demo() :
    // default settings, most can be modified through command line options (see below)
    m_tagDetector(NULL),
    m_tagCodes(AprilTags::tagCodes36h11),

    m_draw(true),
    m_arduino(false),

    m_width(640),
    m_height(480),
    m_tagSize(0.166),
    m_fx(600),
    m_fy(600),
    m_px(m_width/2),
    m_py(m_height/2),

    m_deviceId(0)
  {}

  // changing the tag family
  void setTagCodes(string s) {
    if (s=="16h5") {
      m_tagCodes = AprilTags::tagCodes16h5;
    } else if (s=="25h7") {
      m_tagCodes = AprilTags::tagCodes25h7;
    } else if (s=="25h9") {
      m_tagCodes = AprilTags::tagCodes25h9;
    } else if (s=="36h9") {
      m_tagCodes = AprilTags::tagCodes36h9;
    } else if (s=="36h11") {
      m_tagCodes = AprilTags::tagCodes36h11;
    } else {
      cout << "Invalid tag family specified" << endl;
      exit(1);
    }
  }

  // parse command line options to change default behavior
  void parseOptions(int argc, char* argv[]) {
    int c;
    while ((c = getopt(argc, argv, ":h?adC:F:H:S:W:")) != -1) {
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
      case 'a':
        m_arduino = true;
        break;
      case 'd':
        m_draw = false;
        break;
      case 'C':
        setTagCodes(optarg);
        break;
      case 'F':
        m_fx = atof(optarg);
        m_fy = m_fx;
        m_px = m_fx/2.;
        m_py = m_fy/2.;
        break;
      case 'H':
        m_height = atoi(optarg);
        break;
      case 'S':
        m_tagSize = atof(optarg);
        break;
      case 'W':
        m_width = atoi(optarg);
        break;
      case ':': // unknown option, from getopt
        cout << intro;
        cout << usage;
        exit(1);
        break;
      }
    }

    if (argc == optind + 1) {
      m_deviceId = atoi(argv[optind]);
    }
  }

  void setup() {
    m_tagDetector = new AprilTags::TagDetector(m_tagCodes);

    // find and open a USB camera (built in laptop camera, web cam etc)
    m_cap = cv::VideoCapture(m_deviceId);
        if(!m_cap.isOpened()) {
      cerr << "ERROR: Can't find video device " << m_deviceId << "\n";
      exit(1);
    }
    m_cap.set(CV_CAP_PROP_FRAME_WIDTH, m_width);
    m_cap.set(CV_CAP_PROP_FRAME_HEIGHT, m_height);
    cout << "Camera successfully opened (ignore error messages above...)" << endl;
    cout << "Actual resolution: "
         << m_cap.get(CV_CAP_PROP_FRAME_WIDTH) << "x"
         << m_cap.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;

    // prepare window for drawing the camera images
    if (m_draw) {
      cv::namedWindow(window_name, 1);
    }

    // optional: prepare serial port for communication with Arduino
    if (m_arduino) {
      m_serial.open("/dev/ttyACM0");
    }
  }

  // recovering rotation and translation from the T matrix;
  // converting from camera frame (z forward, x right, y down) to
  // object frame (x forward, y left, z up)
  void getTranslationRotation(const Eigen::Matrix4d& T,
                              Eigen::Vector3d& trans, Eigen::Matrix3d& rot) {
    Eigen::Matrix4d M;
    M <<
      0,  0, 1, 0,
      -1, 0, 0, 0,
      0, -1, 0, 0,
      0,  0, 0, 1;
    Eigen::Matrix4d MT = M*T;
    // translation vector from camera to the April tag
    trans = MT.col(3).head(3);
    // orientation of April tag with respect to camera
    rot = MT.block(0,0,3,3);
  }

  void print_detection(AprilTags::TagDetection& detection) {
    cout << "  Id: " << detection.id
         << " (Hamming: " << detection.hammingDistance << ")";

    // recovering the relative pose of a tag:

    // NOTE: for this to be accurate, it is necessary to use the
    // actual camera parameters here as well as the actual tag size

    Eigen::Matrix4d T =
      detection.getRelativeTransform(m_tagSize, m_fx, m_fy, m_px, m_py);

    // note that for SLAM application it is better to use
    // reprojection error of corner points, as the noise in this
    // relative pose is very non-Gaussian; see iSAM source code for
    // suitable factors

    Eigen::Vector3d translation;
    Eigen::Matrix3d rotation;
    getTranslationRotation(T, translation, rotation);

    cout << "  distance=" << translation.norm()
         << "m, x=" << translation(0)
         << ", y=" << translation(1)
         << ", z=" << translation(2)
         << endl;
  }

  // the processing loop where images are retrieved, tags detected,
  // and information about detections generated
  void loop() {

    cv::Mat image;
    cv::Mat image_gray;

    int frame = 0;
    double last_t = tic();
    while (true) {

      // capture frame
      m_cap >> image;

      // detect April tags (requires a gray scale image)
      cv::cvtColor(image, image_gray, CV_BGR2GRAY);
      vector<AprilTags::TagDetection> detections = m_tagDetector->extractTags(image_gray);

      // print out each detections
      cout << detections.size() << " tags detected:" << endl;
      vector<int> detected_ids;
      for (int i=0; i<detections.size(); i++) {
        print_detection(detections[i]);
        detected_ids.push_back(detections[i].id);
      }

      // draw the current image including any detections
      if (m_draw) {
        for (int i=0; i<detections.size(); i++) {
          // also highlight in the image
          draw_detection(image, detections[i]);
        }
        imshow(window_name, image);
      }

      // optionally send tag information to serial port
      if (m_arduino) {
        if (detected_ids.size() > 0) {
          // only the first detected tag is sent out for now
          m_serial.print(detected_ids[0]);
          m_serial.print(",");
          m_serial.print(0.);
          m_serial.print(",");
          m_serial.print(0.);
          m_serial.print(",");
          m_serial.print(0.);
          m_serial.print("\n");
        } else {
          // no tag deteced: tag ID = -1
          m_serial.print("-1,0.0,0.0,0.0\n");
        }
      }

      // print out the frame rate at which image frames are being processed
      frame++;
      if (frame % 10 == 0) {
        double t = tic();
        cout << "  " << 10./(t-last_t) << " fps" << endl;
        last_t = t;
      }

      // exit if any key is pressed
      if (cv::waitKey(1) >= 0) break;
    }
  }

}; // Demo


// here is were everything begins
int main(int argc, char* argv[]) {
  Demo demo;

  // process command line options
  demo.parseOptions(argc, argv);

  // setup image source, window for drawing, serial port...
  demo.setup();

  // the actual processing loop where tags are detected and visualized
  demo.loop();

  return 0;
}
