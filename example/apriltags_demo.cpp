/**
 * @file april_tags.cpp
 * @brief Example application for April tags library
 * @author: Michael Kaess
 *
 * Opens the first available camera (typically a built in camera in a
 * laptop) and continuously detects April tags in the incoming
 * images. Detections are both shown in the live image and in the text
 * console. Optionally allows selecting of specific camera if multiple
 * are present, specifying image resolution as long as supported by
 * the camera. Also includes the option to send tag detections via a
 * serial port, for example when running on a Raspberry Pi that is
 * connected to an Arduino.
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
    "  -d           disable graphics\n"
    "  -W           image width (availability depends on camera)\n"
    "  -H           image height (availability depends on camera)\n"
    "  -a           Arduino (send tag ids over serial port)\n"
    "\n";

const string intro = "\n"
    "April tags test code\n"
    "(C) 2012-2013 Massachusetts Institute of Technology\n"
    "Michael Kaess\n"
    "\n";


// For Arduino: serial port access
#include <fcntl.h>
#include <termios.h>

// OpenCV library for easy access to USB camera and drawing of images
// on screen
#include "opencv2/opencv.hpp"

// The actual Apriltag detector and the specific family of tags we are
// using
#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag36h11.h"

// Needed for getopt / command line options processing
#include <unistd.h>
extern int optind;
extern char *optarg;


const char* window_name = "apriltags_demo";


// draw April tag detection on actual image
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

private:

  bool draw;
  bool arduino;
  int width;
  int height;
  int device_id;
  cv::VideoCapture cap;
  AprilTags::TagDetector tag_detector;
  int serialPort; // file description for the serial port

public:

  // default constructor
  Demo() :
    draw(true),
    arduino(false),
    width(640),
    height(480),
    device_id(0),
    tag_detector(AprilTags::tagCodes36h11), // determines which family of April tags is detected
    serialPort(-1)
  {}

  void process_options(int argc, char* argv[]) {
    int c;
    while ((c = getopt(argc, argv, ":h?dW:H:a")) != -1) {
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
      case 'a':
        arduino = true;
        break;
      case ':': // unknown option, from getopt
        cout << intro;
        cout << usage;
        exit(1);
        break;
      }
    }

    if (argc == optind + 1) {
      device_id = atoi(argv[optind]);
    }

  }

  void setup() {

    // find and open a USB camera (built in laptop camera, web cam etc)
    cap = cv::VideoCapture(device_id);
        if(!cap.isOpened()) {
      cerr << "ERROR: Can't find video device " << device_id << "\n";
      exit(1);
    }
    cap.set(CV_CAP_PROP_FRAME_WIDTH, width);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, height);
    cout << "Camera successfully opened (ignore error messages above...)" << endl;
    cout << "Actual resolution: "
         << cap.get(CV_CAP_PROP_FRAME_WIDTH) << "x"
         << cap.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;

    // prepare window for drawing the camera images
    if (draw) {
      cv::namedWindow(window_name, 1);
    }

    // optional: prepare serial port for communication with Arduino
    if (arduino) {
      serialPort = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY);
      if (serialPort==-1) {
        cout << "Unable to open serial port" << endl;
        exit(1);
      }
      fcntl(serialPort, F_SETFL,0);

      struct termios port_settings;      // structure to store the port settings in

      cfsetispeed(&port_settings, B9600);    // set baud rates
      cfsetospeed(&port_settings, B9600);

      port_settings.c_cflag &= ~PARENB;    // set no parity, stop bits, data bits
      port_settings.c_cflag &= ~CSTOPB;
      port_settings.c_cflag &= ~CSIZE;
      port_settings.c_cflag |= CS8;
	
      tcsetattr(serialPort, TCSANOW, &port_settings);    // apply the settings to the port
    }

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
      cap >> image;

      // detect Apriltags (requires a gray scale image)
      cv::cvtColor(image, image_gray, CV_BGR2GRAY);
      vector<AprilTags::TagDetection> detections = tag_detector.extractTags(image_gray);

      // print out detections
      cout << detections.size() << " tags detected:" << endl;
      vector<int> detected_ids;
      for (int i=0; i<detections.size(); i++) {

        cout << "  Id: " << detections[i].id
             << " (Hamming: " << detections[i].hammingDistance << ")";

        detected_ids.push_back(detections[i].id);

        if (draw) {
          // also highlight in the image
          draw_detection(image, detections[i]);
        }

        // recovering the relative pose of a tag:

        // NOTE: for this to be accurate, it is necessary to use the
        // actual camera parameters here as well as the actual tag size

        const double tag_size = 0.166; // real side length in meters of square black frame
        const double fx = 600; // camera focal length
        const double fy = 600;
        const double px = image.cols/2; // camera principal point
        const double py = image.rows/2;
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

      if (arduino) {
        int id = -1;
        if (detected_ids.size() > 0) {
          // write the ID number of the first detected tag to a string
          stringstream stream;
          stream << detected_ids[0] << endl;
          string s = stream.str();
          // send the string out to the serial port
          int res = write(serialPort, s.c_str(), s.length());
        } else {
          // negative number means no tag detected
          int res = write(serialPort, "-1\n", 3);
        }
      }

      // draw the current image including any detections
      if (draw) {
        imshow(window_name, image);
      }

      // print out the speed at which image frames are processed
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

};

// here is were everything begins
int main(int argc, char* argv[]) {
  Demo demo;

  // process command line options
  demo.process_options(argc, argv);

  // setup image source, window for drawing, serial port...
  demo.setup();

  // the actual processing loop where tags are detected and visualized
  demo.loop();

  return 0;
}
