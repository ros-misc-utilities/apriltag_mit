
#include "opencv2/opencv.hpp"

#include "TagDetection.h"
#include "MathUtil.h"

#ifdef PLATFORM_APERIOS
//missing/broken isnan
namespace std {
	static bool isnan(float x) {
		const int EXP  = 0x7f800000;
		const int FRAC = 0x007fffff;
		const int y = *((int*)(&x));
		return ((y&EXP)==EXP && (y&FRAC)!=0);
	}
}
#endif

namespace AprilTags {

TagDetection::TagDetection() 
  : good(false), obsCode(), code(), id(), hammingDistance(), rotation(), p(),
    cxy(), observedPerimeter(), homography(), hxy() {
  homography.setZero();
}

TagDetection::TagDetection(int _id)
  : good(false), obsCode(), code(), id(_id), hammingDistance(), rotation(), p(),
    cxy(), observedPerimeter(), homography(), hxy() {
  homography.setZero();
}

float TagDetection::getXYOrientation() const {
  // Because the order of segments in a quad is arbitrary, so is the
  // homography's rotation, so we can't determine orientation directly
  // from the homography.  Instead, use the homography to find two
  // bottom corners of a properly oriented tag in pixel coordinates,
  // and then compute orientation from that.
  std::pair<float,float> p0 = interpolate(-1,-1);   // lower left corner of tag
  std::pair<float,float> p1 = interpolate(1,-1);    // lower right corner of tag
  float orient = atan2(p1.second - p0.second, p1.first - p0.first);
  return ! std::isnan(float(orient)) ? orient : 0.;
}

std::pair<float,float> TagDetection::interpolate(float x, float y) const {
  float z = homography(2,0)*x + homography(2,1)*y + homography(2,2);
  if ( z == 0 )
    return std::pair<float,float>(0,0);  // prevents returning a pair with a -NaN, for which gcc 4.4 flubs isnan
  float newx = (homography(0,0)*x + homography(0,1)*y + homography(0,2))/z + hxy.first;
  float newy = (homography(1,0)*x + homography(1,1)*y + homography(1,2))/z + hxy.second;
  return std::pair<float,float>(newx,newy);
}

bool TagDetection::overlapsTooMuch(const TagDetection &other) const {
  // Compute a sort of "radius" of the two targets. We'll do this by
  // computing the average length of the edges of the quads (in
  // pixels).
  float radius =
    ( MathUtil::distance2D(p[0], p[1]) +
      MathUtil::distance2D(p[1], p[2]) +
      MathUtil::distance2D(p[2], p[3]) +
      MathUtil::distance2D(p[3], p[0]) +
      MathUtil::distance2D(other.p[0], other.p[1]) +
      MathUtil::distance2D(other.p[1], other.p[2]) +
      MathUtil::distance2D(other.p[2], other.p[3]) +
      MathUtil::distance2D(other.p[3], other.p[0]) ) / 16.0f;

  // distance (in pixels) between two tag centers
  float dist = MathUtil::distance2D(cxy, other.cxy);

  // reject pairs where the distance between centroids is smaller than
  // the "radius" of one of the tags.
  return ( dist < radius );
}

Eigen::Matrix4d TagDetection::getRelativeTransform(double tag_size, double fx, double fy, double px, double py) {
  std::vector<cv::Point3f> objPts;
  std::vector<cv::Point2f> imgPts;
  double s = tag_size/2.;
  objPts.push_back(cv::Point3f(-s,-s, 0));
  objPts.push_back(cv::Point3f( s,-s, 0));
  objPts.push_back(cv::Point3f( s, s, 0));
  objPts.push_back(cv::Point3f(-s, s, 0));

  std::pair<float, float> p1 = p[0];
  std::pair<float, float> p2 = p[1];
  std::pair<float, float> p3 = p[2];
  std::pair<float, float> p4 = p[3];
  imgPts.push_back(cv::Point2f(p1.first, p1.second));
  imgPts.push_back(cv::Point2f(p2.first, p2.second));
  imgPts.push_back(cv::Point2f(p3.first, p3.second));
  imgPts.push_back(cv::Point2f(p4.first, p4.second));

  cv::Mat rvec, tvec;
  cv::Matx33f cameraMatrix(
                           fx, 0, px,
                           0, fy, py,
                           0,  0,  1);
  cv::Vec4f distParam(0,0,0,0); // all 0?
  cv::solvePnP(objPts, imgPts, cameraMatrix, distParam, rvec, tvec);
  cv::Matx33d r;
  Rodrigues(rvec, r);
  Eigen::Matrix3d wRo;
  wRo << r(0,0), r(0,1), r(0,2), r(1,0), r(1,1), r(1,2), r(2,0), r(2,1), r(2,2);

  Eigen::Matrix4d T; 
  T.topLeftCorner(3,3) = wRo;
  T.col(3).head(3) << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
  T.row(3) << 0,0,0,1;

  return T;
}

} // namespace
